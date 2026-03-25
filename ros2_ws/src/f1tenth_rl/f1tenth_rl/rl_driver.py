import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

# 仮想環境のライブラリパスを動的に追加 (WSL2/Jetson 共用)
home_dir = os.path.expanduser('~')
VENV_PATH = os.path.join(home_dir, 'projects/jetson-ros2-project/jetson-ros2/lib/python3.10/site-packages')
if os.path.exists(VENV_PATH) and VENV_PATH not in sys.path:
    sys.path.append(VENV_PATH)

from stable_baselines3 import PPO

from .utils import LidarProcessor

class RLDriver(Node):

    def __init__(self):
        super().__init__('rl_driver')
        self.get_logger().info("RL Driver node starting...")

        # --- パラメータ宣言 ---
        home_dir = os.path.expanduser("~")
        df_model = os.path.join(home_dir, 'projects/jetson-ros2-project/ros2_ws/models/model')
        self.declare_parameter('model_path', df_model)
        
        # LiDAR 前処理
        self.declare_parameter('lidar_num_beams', 108)
        self.declare_parameter('lidar_downsample_step', 10)
        self.declare_parameter('lidar_center_crop', True)
        
        # Sim-to-Real 用
        self.declare_parameter('use_sim_to_real', True)
        self.declare_parameter('lidar_noise_std', 0.02)
        self.declare_parameter('steer_smoothing', 0.5)
        self.declare_parameter('speed_smoothing', 0.8)
        
        # 安全レイヤー
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_dist', 0.3)
        self.declare_parameter('safety_check_width_percent', 0.15) # 前方中央の何%をチェックするか

        # --- モデルの読み込み ---
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not os.path.isabs(model_path):
            model_path = os.path.join(home_dir, model_path)
            
        try:
            if not os.path.exists(model_path + ".zip") and not os.path.exists(model_path):
                 self.get_logger().error(f"MODEL NOT FOUND: {model_path}")
                 self.model = None
            else:
                self.model = PPO.load(model_path)
                self.get_logger().info(f"Loaded RL model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            self.model = None

        # 2. 内部状態
        self.speed = 0.0
        self.steer = 0.0
        self.last_steer = 0.0
        self.last_speed = 0.0

        # LiDAR 前処理クラスの初期化
        self.processor = LidarProcessor(
            num_beams=self.get_parameter('lidar_num_beams').value,
            downsample_step=self.get_parameter('lidar_downsample_step').value,
            center_crop=self.get_parameter('lidar_center_crop').value,
            noise_std=self.get_parameter('lidar_noise_std').value if self.get_parameter('use_sim_to_real').value else 0.0
        )

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

        # 3. AI 推論用の入力作成 [LiDAR, Speed, Steer]
        state = np.concatenate([lidar, np.array([self.speed, self.steer], dtype=np.float32)])
        
        # モデル推論
        action, _ = self.model.predict(state)

        # 4. アクション決定 (モデルの出力次元に合わせて調整)
        pred_speed = float(action[0]) if len(action) >= 2 else (1.0 if len(action) == 1 else 0.0)
        pred_steer = float(action[1]) if len(action) >= 2 else (float(action[0]) if len(action) == 1 else 0.0)

        # 5. Sim-to-Real: EMA によるアクションの平滑化
        if self.get_parameter('use_sim_to_real').value:
            s_alpha = self.get_parameter('steer_smoothing').value
            v_alpha = self.get_parameter('speed_smoothing').value
            pred_steer = s_alpha * pred_steer + (1.0 - s_alpha) * self.last_steer
            pred_speed = v_alpha * pred_speed + (1.0 - v_alpha) * self.last_speed
            
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
