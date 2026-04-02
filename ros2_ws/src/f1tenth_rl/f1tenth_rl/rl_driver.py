import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np



from stable_baselines3 import PPO
import onnxruntime as ort

from .utils import LidarProcessor

class RLDriver(Node):

    def __init__(self):
        super().__init__('rl_driver')
        self.get_logger().info("RL Driver node starting...")

        # --- パラメータ宣言 ---
        self.declare_parameter('model_path', 'models/model')
        
        # --- 正規化パラメータ (JSONがない場合のフォールバック用) ---
        self.declare_parameter('obs_lidar_mean', 4.869)
        self.declare_parameter('obs_lidar_std', 3.577)
        self.declare_parameter('obs_lidar_residual_mean', -0.008)
        self.declare_parameter('obs_lidar_residual_std', 0.084)
        self.declare_parameter('obs_speed_mean', 0.574)
        self.declare_parameter('obs_speed_std', 0.096)
        self.declare_parameter('obs_steer_mean', -0.010)
        self.declare_parameter('obs_steer_std', 0.122)
        
        # LiDAR 前処理
        # LiDAR 前処理設定
        # num_beams: 108 (通常) または 216 (ΔLiDAR/Residual使用時) などモデルに合わせる
        self.declare_parameter('lidar_num_beams', 108)
        self.declare_parameter('lidar_downsample_step', 10) # 1080 -> 108 次元
        self.declare_parameter('lidar_center_crop', True) # 前方中央を抽出
        self.declare_parameter('lidar_median_filter_size', 5) # スパイクノイズ除去用フィルタ
        
        # Sim-to-Real / 走行安定化設定
        # use_sim_to_real: 平滑化(EMA)やスルーレート制限を有効にするフラグ
        self.declare_parameter('use_sim_to_real', True)
        self.declare_parameter('lidar_noise_std', 0.02) # 検証用に追加する疑似ノイズ量
        
        # 指数移動平均(EMA)係数: 小さいほど滑らか(低速反応)、大きいほど即座に反応
        self.declare_parameter('steer_smoothing', 0.3)
        self.declare_parameter('speed_smoothing', 0.4)
        
        # 1ステップあたりの最大変化量 (クランクや急加減速でのスリップ・機器損傷を防止)
        self.declare_parameter('max_steer_change_rate', 0.15)
        self.declare_parameter('max_speed_change_rate', 0.2)
        # モーターのガタつき防止のための最小速度閾値
        self.declare_parameter('speed_deadband', 0.05)
        
        # 安全レイヤー: 前方中央の特定の幅に障害物があれば即停止
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_dist', 0.3) # 停止距離(m)
        self.declare_parameter('safety_check_width_percent', 0.15) # 視界の何%を障害物検知に使うか

        # --- モデルの読み込み ---
        model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
        if model_path_param.startswith("~/"):
            model_path = os.path.expanduser(model_path_param)
        elif not os.path.isabs(model_path_param):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('f1tenth_rl')
                workspace_root = os.path.abspath(os.path.join(pkg_share, '../../..'))
                # ROS2 workspace root (e.g. jetson-ros2-project)
                model_path = os.path.join(workspace_root, model_path_param)
                if not os.path.exists(os.path.dirname(model_path)):
                     model_path = os.path.abspath(model_path_param)
            except Exception:
                model_path = os.path.abspath(model_path_param)
        else:
            model_path = model_path_param
            
        try:
            # 拡張子を確認
            if model_path.endswith('.onnx'):
                self.model_type = 'onnx'
                self.ort_session = ort.InferenceSession(model_path)
                self.get_logger().info(f"Loaded ONNX model: {model_path}")
                # 入力次元の確認
                self.input_dim = self.ort_session.get_inputs()[0].shape[1]
                self.get_logger().info(f"Model Input Dimension: {self.input_dim}")
            else:
                self.model_type = 'sb3'
                if not os.path.exists(model_path + ".zip") and not os.path.exists(model_path):
                     # ファイルがない場合は .zip を付けて再試行
                     if os.path.exists(model_path + ".zip"):
                         model_path += ".zip"
                     else:
                         self.get_logger().error(f"MODEL NOT FOUND: {model_path}")
                         self.model = None
                
                if model_path:
                    self.model = PPO.load(model_path, device="cpu")
                    self.get_logger().info(f"Loaded SB3 model: {model_path}")
                    self.input_dim = self.model.observation_space.shape[0]
                else:
                    self.model = None
                    self.input_dim = 0
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            self.model = None
            self.model_type = None
            self.input_dim = 0

        # 2. 内部状態
        self.speed = 0.0
        self.steer = 0.0
        self.last_steer = 0.0
        self.last_speed = 0.0
        
        # LiDAR Residual (ΔLiDAR) 用
        self.prev_lidar = None

        # LiDAR 前処理クラスの初期化
        self.processor = LidarProcessor(
            num_beams=self.get_parameter('lidar_num_beams').value,
            downsample_step=self.get_parameter('lidar_downsample_step').value,
            center_crop=self.get_parameter('lidar_center_crop').value,
            noise_std=self.get_parameter('lidar_noise_std').value if self.get_parameter('use_sim_to_real').value else 0.0,
            median_filter_size=self.get_parameter('lidar_median_filter_size').value
        )
        
        # --- 正規化定数の読み込み ---
        self.NORMALIZE_OBSERVATIONS = True
        
        self.LIDAR_MEAN = self.get_parameter('obs_lidar_mean').value
        self.LIDAR_STD = self.get_parameter('obs_lidar_std').value
        self.LIDAR_RESIDUAL_MEAN = self.get_parameter('obs_lidar_residual_mean').value
        self.LIDAR_RESIDUAL_STD = self.get_parameter('obs_lidar_residual_std').value
        
        self.VEHICLE_STATE_MEAN = np.array([
            self.get_parameter('obs_speed_mean').value,
            self.get_parameter('obs_steer_mean').value
        ], dtype=np.float32)
        self.VEHICLE_STATE_STD = np.array([
            self.get_parameter('obs_speed_std').value,
            self.get_parameter('obs_steer_std').value
        ], dtype=np.float32)

        # 3. ROS 通信
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.steer = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        if self.model is None:
            return

        # 1. LiDAR 前処理 (Processor に委譲)
        lidar = self.processor.process(msg.ranges, msg.range_max)

        # 2. 安全レイヤー (前方障害物検知)
        if self.get_parameter('safety_enable').value:
            stop_dist = self.get_parameter('safety_stop_dist').value
            check_width_percent = self.get_parameter('safety_check_width_percent').value
            
            # 中央付近を一定幅チェック
            # widthはLiDARビーム総数に対する割合で計算
            width = max(1, int(self.processor.num_beams * check_width_percent / 2)) # 半分の幅を計算
            mid = self.processor.num_beams // 2
            
            # 範囲をクリップして有効なインデックスのみを使用
            start_idx = max(0, mid - width)
            end_idx = min(self.processor.num_beams, mid + width)
            
            front_min = np.min(lidar[start_idx : end_idx])
            
            if front_min < stop_dist:
                self.get_logger().warn(f"EMERGENCY STOP! Obj at {front_min:.2f}m")
                self.publish_drive(0.0, 0.0)
                return

        # 3. AI 推論用の入力作成
        # モデルの入力次元に応じて LiDAR Residual を含めるか判定
        num_beams = self.processor.num_beams
        
        # 正規化の準備
        lidar_feat = lidar
        vehicle_state_feat = np.array([self.speed, self.steer], dtype=np.float32)
        
        if self.NORMALIZE_OBSERVATIONS:
            lidar_feat = (lidar_feat - self.LIDAR_MEAN) / self.LIDAR_STD
            vehicle_state_feat = (vehicle_state_feat - self.VEHICLE_STATE_MEAN) / self.VEHICLE_STATE_STD

        if self.input_dim == (num_beams * 2 + 2):
            # ΔLiDAR (Residual) を含める (EXP-14 形式)
            if self.prev_lidar is None:
                self.prev_lidar = lidar.copy()
            
            delta_lidar = lidar - self.prev_lidar
            self.prev_lidar = lidar.copy()
            
            delta_feat = delta_lidar
            if self.NORMALIZE_OBSERVATIONS:
                delta_feat = (delta_feat - self.LIDAR_RESIDUAL_MEAN) / self.LIDAR_RESIDUAL_STD
            
            state = np.concatenate([lidar_feat, delta_feat, vehicle_state_feat])
        else:
            # 通常形式 (110次元など)
            state = np.concatenate([lidar_feat, vehicle_state_feat])
            
            # prev_lidar を使用しない場合でも更新しておく
            self.prev_lidar = lidar.copy()
        
        # モデル推論
        if self.model_type == 'onnx':
            # ONNX Runtime 推論 [1, Dim]
            ort_inputs = {self.ort_session.get_inputs()[0].name: state.reshape(1, -1).astype(np.float32)}
            ort_outputs = self.ort_session.run(None, ort_inputs)
            action = ort_outputs[0][0] # [Batch=1, Action=2] -> [2]
        elif self.model_type == 'sb3' and self.model:
            action, _ = self.model.predict(state, deterministic=True)
        else:
            return

        # 4. アクション決定 (モデルの出力次元に合わせて調整)
        pred_speed = float(action[0]) if len(action) >= 2 else (1.0 if len(action) == 1 else 0.0)
        pred_steer = float(action[1]) if len(action) >= 2 else (float(action[0]) if len(action) == 1 else 0.0)

        # 5. Sim-to-Real: 物理制約に基づく平滑化と安全処理
        if self.get_parameter('use_sim_to_real').value:
            # 5-1. 微小速度のカット (Deadband)
            deadband = self.get_parameter('speed_deadband').value
            if abs(pred_speed) < deadband:
                pred_speed = 0.0

            # 5-2. スルーレート制限 (急激な変化の防止)
            max_s_diff = self.get_parameter('max_steer_change_rate').value
            max_v_diff = self.get_parameter('max_speed_change_rate').value
            
            s_diff = np.clip(pred_steer - self.last_steer, -max_s_diff, max_s_diff)
            v_diff = np.clip(pred_speed - self.last_speed, -max_v_diff, max_v_diff)
            
            pred_steer_clipped = self.last_steer + s_diff
            pred_speed_clipped = self.last_speed + v_diff

            # 5-3. 指数移動平均 (EMA) による全体的な平滑化
            s_alpha = self.get_parameter('steer_smoothing').value
            v_alpha = self.get_parameter('speed_smoothing').value
            pred_steer = s_alpha * pred_steer_clipped + (1.0 - s_alpha) * self.last_steer
            pred_speed = v_alpha * pred_speed_clipped + (1.0 - v_alpha) * self.last_speed
            
        self.last_steer, self.last_speed = pred_steer, pred_speed

        # 6. 指令値をパブリッシュ
        self.publish_drive(pred_speed, pred_steer)

    def publish_drive(self, speed, steer):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
