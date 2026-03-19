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
            # SB3は拡張子 .zip を自動付与するため、ここでは拡張子なしのパスを渡すか、
            # あるいは明示的にチェックする
            self.model = PPO.load(model_path)
            self.get_logger().info(f"Loaded RL model from {model_path}")
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

        # 5. モデル入力サイズに合わせるため、データが少なければパディング、多ければカット
        TARGET_SIZE = 1080
        if len(lidar) >= TARGET_SIZE:
            lidar = lidar[:TARGET_SIZE]
        else:
            padding_size = TARGET_SIZE - len(lidar)
            lidar = np.pad(lidar, (0, padding_size), 'constant', constant_values=(msg.range_max,))

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

        # 指令値をPublish
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
