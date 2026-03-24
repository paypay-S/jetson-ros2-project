import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from stable_baselines3 import PPO

class RLDriver(Node):

    def __init__(self):
        super().__init__('rl_driver')

        self.get_logger().info("RL Driver node started")

        # 1. 絶対パス等で指定できるよう、モデルパスをParameter化
        self.declare_parameter('model_path', '/home/toyonishiorin/f1tenth-project/ros2_ws/models/model')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        # モデル読み込み
        try:
            self.model = PPO.load(model_path)
            self.get_logger().info(f"Loaded RL model from {model_path}")
            
            # 1.1 モデルの入出力次元を検証
            obs_shape = self.model.observation_space.shape[0]
            act_shape = self.model.action_space.shape[0]
            self.get_logger().info(f"Model Space: Observation={obs_shape}, Action={act_shape}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {model_path}: {e}")
            self.model = None

        # 状態用
        self.speed = 0.0
        self.steer = 0.0

        # LiDAR subscribe
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        
        # 2. 現在の速度・ステアリングをSubscribeして自己位置/速度を更新 (/odom を想定)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        
        # 3. ACK を出せるようにPublisherを追加 (/driveへのpublishを想定)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        # 4. Sim-to-Real 用のパラメータと変数
        self.declare_parameter('use_sim_to_real', True)
        self.declare_parameter('lidar_noise_std', 0.02)     # LiDARへのガウスノイズ (m)
        self.declare_parameter('steer_smoothing', 0.5)      # 0.0~1.0 (大きいほど新しい値を優先)
        self.declare_parameter('speed_smoothing', 0.8)      # 速度の平滑化
        
        # 5. LiDARの前処理パラメータ (可変次元対応)
        self.declare_parameter('lidar_num_beams', 108)      # モデルに入力するLiDARの次元数
        self.declare_parameter('lidar_downsample_step', 10) # 間引き間隔
        self.declare_parameter('lidar_center_crop', True)   # 中心を基準とした切り出しを行うか
        
        # 8. 安全レイヤー (前方衝突検知)
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_dist', 0.3)     # 緊急停止距離 (m)

        self.last_steer = 0.0
        self.last_speed = 0.0

    def odom_callback(self, msg):
        # 現在の走行速度を更新
        self.speed = msg.twist.twist.linear.x
        # Odomからの角速度等で推定可能だが、今回は一旦angular.zを使用
        self.steer = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        if self.model is None:
            return

        # LiDAR取得
        lidar = np.array(msg.ranges, dtype=np.float32)

        # NaN / inf を除去
        lidar = np.nan_to_num(
            lidar,
            nan=msg.range_max,
            posinf=msg.range_max,
            neginf=0.0
        )

        # 5. ダウンサンプリングとサイズ調整 (モデル入力を108次元等に合わせる)
        downsample_step = self.get_parameter('lidar_downsample_step').value
        target_size = self.get_parameter('lidar_num_beams').value
        center_crop = self.get_parameter('lidar_center_crop').value

        # 中心クロップ (前方中心を基準に取り出す)
        if center_crop:
            n = len(lidar)
            # モデルが必要な元の点数 (target * step)
            required_raw = target_size * downsample_step
            if n > required_raw:
                start_idx = (n - required_raw) // 2
                lidar = lidar[start_idx : start_idx + required_raw]

        # 間引き処理
        if downsample_step > 1:
            lidar = lidar[::downsample_step]

        # モデルの入力次元に合わせる
        if len(lidar) >= target_size:
            lidar = lidar[:target_size]
        else:
            padding_size = target_size - len(lidar)
            lidar = np.pad(lidar, (0, padding_size), 'constant', constant_values=(msg.range_max,))

        # 8. 安全レイヤー: 前方の障害物検知
        is_emergency = False
        if self.get_parameter('safety_enable').value:
            stop_dist = self.get_parameter('safety_stop_dist').value
            # 切り出したLiDARのうち、中央付近（±30度程度）をチェック
            # ※ 108次元の場合、中心54を基準に±12点程度 (約30度 × 108/270度 = 12)
            check_width = max(1, target_size // 8)
            center_idx = target_size // 2
            front_beams = lidar[center_idx - check_width : center_idx + check_width]
            
            if len(front_beams) > 0 and np.min(front_beams) < stop_dist:
                is_emergency = True
                self.get_logger().warn(f"EMERGENCY STOP! Obstacle detected at {np.min(front_beams):.2f}m")

        # 6. Sim-to-Real: 観測データへのノイズ追加 (堅牢性の向上)
        if self.get_parameter('use_sim_to_real').value:
            noise_std = self.get_parameter('lidar_noise_std').value
            if noise_std > 0:
                noise = np.random.normal(0, noise_std, size=lidar.shape).astype(np.float32)
                lidar = np.clip(lidar + noise, 0.0, msg.range_max)

        # state作成
        state = np.concatenate([
            lidar,
            np.array([self.speed, self.steer], dtype=np.float32)
        ])

        # AI推論
        action, _ = self.model.predict(state)

        # /drive への Message作成と設定
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        
        # ※ 学習済みモデルの Action 出力次元 (速度とステアリングの2次元か、ステアリングのみの1次元か) に応じて処理
        if len(action) >= 2:
            pred_speed = float(action[0])
            pred_steer = float(action[1])
        elif len(action) == 1:
            pred_speed = 1.0  # デフォルト速度
            pred_steer = float(action[0])
        else:
            pred_speed = 0.0
            pred_steer = 0.0

        # 7. Sim-to-Real: アクションの平滑化 (EMA)
        # 急激なステアリング変化によるサーボへの負荷と機体の挙動不整合を抑制
        if self.get_parameter('use_sim_to_real').value:
            s_alpha = self.get_parameter('steer_smoothing').value
            v_alpha = self.get_parameter('speed_smoothing').value
            
            pred_steer = s_alpha * pred_steer + (1.0 - s_alpha) * self.last_steer
            pred_speed = v_alpha * pred_speed + (1.0 - v_alpha) * self.last_speed
            
        self.last_steer = pred_steer
        self.last_speed = pred_speed

        drive_msg.drive.speed = pred_speed
        drive_msg.drive.steering_angle = pred_steer

        # 指令値をPublish (緊急時は上書き)
        if is_emergency:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.last_speed = 0.0
            self.last_steer = 0.0

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
