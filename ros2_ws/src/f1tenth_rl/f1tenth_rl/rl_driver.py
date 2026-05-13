import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from collections import deque
from typing import Optional

from .utils import LidarProcessor
from .managers import ModelManager, RecoveryManager, SafetyLayer

class RLDriver(Node):
    def __init__(self):
        super().__init__('rl_driver')
        self.get_logger().info("RL Driver node starting...")

        self._declare_parameters()
        
        # マネージャーの初期化
        self.model_manager = ModelManager(self.get_logger())
        self.recovery_manager = RecoveryManager(self.get_logger(), self.get_clock())
        self.safety_layer = SafetyLayer(self.get_logger())
        
        # プロセッサの初期化
        self.processor = LidarProcessor(
            num_beams=self.get_parameter('lidar_num_beams').value,
            downsample_step=self.get_parameter('lidar_downsample_step').value,
            center_crop=self.get_parameter('lidar_center_crop').value,
            noise_std=self.get_parameter('lidar_noise_std').value if self.get_parameter('use_sim_to_real').value else 0.0,
            median_filter_size=self.get_parameter('lidar_median_filter_size').value
        )

        # モデルの読み込み
        self._load_model()
        
        # 内部状態
        self.speed = 0.0
        self.steer = 0.0
        self.last_steer = 0.0
        self.last_speed = 0.0
        
        # 正規化定数
        self._init_normalization()

        # Frame Stacking 用のバッファ
        self.frame_stack_size = self.get_parameter('frame_stack').value
        self.obs_buffer = deque(maxlen=self.frame_stack_size)
        self.current_state = None

        # ROS 通信
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

    def _declare_parameters(self):
        self.declare_parameter('model_path', 'models/model')
        # 正規化
        self.declare_parameter('obs_lidar_mean', 4.869)
        self.declare_parameter('obs_lidar_std', 3.577)
        self.declare_parameter('obs_speed_mean', 0.574)
        self.declare_parameter('obs_speed_std', 0.096)
        self.declare_parameter('obs_steer_mean', -0.010)
        self.declare_parameter('obs_steer_std', 0.122)
        # LiDAR
        self.declare_parameter('lidar_num_beams', 108)
        self.declare_parameter('lidar_downsample_step', 10)
        self.declare_parameter('lidar_center_crop', True)
        self.declare_parameter('lidar_median_filter_size', 5)
        # Sim-to-Real
        self.declare_parameter('use_sim_to_real', True)
        self.declare_parameter('lidar_noise_std', 0.02)
        self.declare_parameter('steer_smoothing', 0.3)
        self.declare_parameter('speed_smoothing', 0.4)
        self.declare_parameter('max_steer_change_rate', 0.15)
        self.declare_parameter('max_speed_change_rate', 0.2)
        self.declare_parameter('speed_deadband', 0.05)
        # ゲイン
        self.declare_parameter('speed_multiplier', 1.0)
        self.declare_parameter('steer_multiplier', 1.0)
        # 安全
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_dist', 0.2)
        self.declare_parameter('safety_check_angle', 30.0)
        self.declare_parameter('frame_stack', 4)
        # 復帰
        self.declare_parameter('recovery_enabled', True)
        self.declare_parameter('recovery_stop_dist', 0.3)
        self.declare_parameter('recovery_reverse_speed', -0.5)
        self.declare_parameter('recovery_steer_magnitude', 0.4)
        self.declare_parameter('recovery_brake_duration', 1.0)
        self.declare_parameter('recovery_stop_duration', 1.0)
        self.declare_parameter('recovery_back_duration', 3.0)
        self.declare_parameter('trigger_limit', 5)

    def _init_normalization(self):
        self.LIDAR_MEAN = self.get_parameter('obs_lidar_mean').value
        self.LIDAR_STD = self.get_parameter('obs_lidar_std').value
        self.VEHICLE_STATE_MEAN = np.array([
            self.get_parameter('obs_speed_mean').value,
            self.get_parameter('obs_steer_mean').value
        ], dtype=np.float32)
        self.VEHICLE_STATE_STD = np.array([
            self.get_parameter('obs_speed_std').value,
            self.get_parameter('obs_steer_std').value
        ], dtype=np.float32)

    def _load_model(self):
        model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
        # ワークスペースルートを考慮したパス解決
        if not os.path.isabs(model_path_param) and model_path_param.lower() != "none":
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('f1tenth_rl')
                workspace_root = os.path.abspath(os.path.join(pkg_share, '../../..'))
                model_path = os.path.join(workspace_root, model_path_param)
                if not os.path.exists(model_path) and not os.path.exists(model_path + ".zip"):
                    model_path = os.path.abspath(model_path_param)
            except Exception:
                model_path = os.path.abspath(model_path_param)
        else:
            model_path = model_path_param
        
        self.model_manager.load_model(model_path)

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.steer = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        # 1. LiDAR 前処理
        lidar = self.processor.process(msg.ranges, msg.range_max)
        
        # 2. 状態更新 (バッファ)
        self._update_state(lidar)

        # 3. 復帰動作中かチェック
        speed, steer, active = self.recovery_manager.get_command(
            self.get_parameter('recovery_reverse_speed').value,
            self.get_parameter('recovery_brake_duration').value,
            self.get_parameter('recovery_stop_duration').value,
            self.get_parameter('recovery_back_duration').value
        )
        if active:
            self.publish_drive(speed, steer)
            return

        # 4. 安全レイヤー / 復帰トリガー
        is_collision, dist = self.safety_layer.check_front_collision(
            lidar, self.get_parameter('safety_check_angle').value, 200.0, self.get_parameter('recovery_stop_dist').value
        )
        
        if self.get_parameter('recovery_enabled').value and is_collision:
            self.recovery_manager.trigger_count_wall += 1
            if self.recovery_manager.trigger_count_wall >= self.get_parameter('trigger_limit').value:
                # 復帰開始
                side_steer = self.get_parameter('recovery_steer_magnitude').value
                # 左右の開けた方に逃げる
                mid = len(lidar) // 2
                if np.mean(lidar[:mid]) < np.mean(lidar[mid:]):
                    self.recovery_manager.start(side_steer)
                else:
                    self.recovery_manager.start(-side_steer)
                return
        else:
            self.recovery_manager.trigger_count_wall = 0

        # 緊急停止 (復帰距離より短い場合)
        if self.get_parameter('safety_enable').value and dist < self.get_parameter('safety_stop_dist').value:
            self.get_logger().warn(f"EMERGENCY STOP! Dist: {dist:.2f}m")
            self.publish_drive(0.0, 0.0)
            return

        # 5. モデル推論
        if self.current_state is not None:
            action = self.model_manager.predict(self.current_state)
            if action is not None:
                self._apply_action(action)

    def _update_state(self, lidar):
        lidar_feat = (lidar.astype(np.float32) - self.LIDAR_MEAN) / self.LIDAR_STD
        state_feat = (np.array([self.speed, self.steer], dtype=np.float32) - self.VEHICLE_STATE_MEAN) / self.VEHICLE_STATE_STD
        
        current_obs = np.concatenate([lidar_feat, state_feat])
        self.obs_buffer.append(current_obs)
        while len(self.obs_buffer) < self.frame_stack_size:
            self.obs_buffer.appendleft(current_obs)
        
        self.current_state = np.concatenate(list(self.obs_buffer))

    def _apply_action(self, action):
        pred_speed = float(action[0]) if len(action) >= 2 else (1.0 if len(action) == 1 else 0.0)
        pred_steer = float(action[1]) if len(action) >= 2 else (float(action[0]) if len(action) == 1 else 0.0)

        # ゲイン
        pred_speed *= self.get_parameter('speed_multiplier').value
        pred_steer *= self.get_parameter('steer_multiplier').value

        # Sim-to-Real 処理
        if self.get_parameter('use_sim_to_real').value:
            # デッドバンド
            if abs(pred_speed) < self.get_parameter('speed_deadband').value:
                pred_speed = 0.0
            
            # スルーレート & EMA
            max_s_diff = self.get_parameter('max_steer_change_rate').value
            max_v_diff = self.get_parameter('max_speed_change_rate').value
            s_alpha = self.get_parameter('steer_smoothing').value
            v_alpha = self.get_parameter('speed_smoothing').value
            
            s_clipped = self.last_steer + np.clip(pred_steer - self.last_steer, -max_s_diff, max_s_diff)
            v_clipped = self.last_speed + np.clip(pred_speed - self.last_speed, -max_v_diff, max_v_diff)
            
            pred_steer = s_alpha * s_clipped + (1.0 - s_alpha) * self.last_steer
            pred_speed = v_alpha * v_clipped + (1.0 - v_alpha) * self.last_speed
            
        self.last_steer, self.last_speed = pred_steer, pred_speed
        self.publish_drive(pred_speed, pred_steer)

    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

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
