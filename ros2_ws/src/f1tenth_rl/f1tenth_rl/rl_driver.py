import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from collections import deque
from typing import Optional

from .utils import LidarProcessor
from .managers import ModelManager
from .model_health import ModelHealthMonitor
from .safety import SafetyManager
from .collision_recovery import CollisionRecoverySequence
from .racing_line import RacingLine
from .pure_pursuit import PurePursuitController
from tf2_ros import Buffer, TransformListener


class RLDriver(Node):
    def __init__(self):
        super().__init__('rl_driver')
        self.get_logger().info("RL Driver node starting for 458-dimensional model...")

        self._declare_parameters()

        # マネージャーの初期化
        self.model_manager = ModelManager(self.get_logger())
        
        # 新しい安全管理システムの初期化（Phase 1コンポーネント）
        self.model_health_monitor = ModelHealthMonitor(timeout_sec=self.get_parameter('model_inference_timeout').value)

        # プロセッサの初期化
        self.processor = LidarProcessor(
            num_beams=self.get_parameter('lidar_num_beams').value,
            downsample_step=self.get_parameter('lidar_downsample_step').value,
            center_crop=self.get_parameter('lidar_center_crop').value,
            median_filter_size=self.get_parameter('lidar_median_filter_size').value
        )

        # モデルの読み込み
        self._load_model()

        # 内部状態
        self.speed = 0.0
        self.steer = 0.0
        self.last_steer = 0.0
        self.last_speed = 0.0

        # ワールド座標系での位置・姿勢
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        # 行動履歴 [prev_steer_norm, prev_speed_norm]
        self.prev_action = np.zeros(2, dtype=np.float32)

        # レーシングラインおよび Pure Pursuit
        rl_path = self.get_parameter('racing_line_path').value
        if not rl_path:
            # デフォルトで maps ディレクトリの下の centerline CSV を自動探索する
            pkg_share = ""
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('f1tenth_rl')
                workspace_root = os.path.abspath(os.path.join(pkg_share, '../../..'))
                # maps ディレクトリ直下の yaml と同じ名前の csv または最初の csv を探す
                maps_dir = os.path.join(workspace_root, "maps")
                if os.path.exists(maps_dir):
                    csv_files = [f for f in os.listdir(maps_dir) if f.endswith(".csv")]
                    if csv_files:
                        rl_path = os.path.join(maps_dir, csv_files[0])
            except Exception:
                pass

        self.get_logger().info(f"Loading RacingLine CSV: {rl_path if rl_path else 'None'}")
        self.racing_line = RacingLine(rl_path)
        self.pp_controller = PurePursuitController(
            self.racing_line,
            wheelbase=0.3255,  # 車両ホイールベース
            lookahead_dist=0.8
        )
        
        # 衝突復帰シーケンスと安全管理システムの初期化
        self.collision_recovery = CollisionRecoverySequence(
            self.pp_controller,
            backing_time=1.5,
            turning_time=2.0,
            moving_forward_time=1.0
        )
        self.safety_manager = SafetyManager(
            self.model_health_monitor,
            self.pp_controller,
            self.collision_recovery,
            fallback_timeout_sec=10.0,
            collision_recovery_timeout_sec=5.0
        )
        # 既存パラメータとの互換性を保つため、safety_manager のパラメータを設定
        self.safety_manager.collision_check_angle_deg = 60.0  # 前方 ±30°
        self.safety_manager.collision_total_fov_deg = 200.0   # 既存コードの値に合わせる
        self.safety_manager.collision_stop_dist = 0.3  # デフォルト

        # Frame Stacking / Frame Skipping 用のバッファ
        self.frame_stack_size = self.get_parameter('frame_stack').value
        self.frame_skip = self.get_parameter('frame_skip').value
        # f1_env.py の (FRAME_STACK - 1) * FRAME_SKIP + 1 に合わせる
        maxlen = (self.frame_stack_size - 1) * self.frame_skip + 1
        self.obs_buffer = deque(maxlen=maxlen)
        self.current_state = None

        # ROS 通信
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # TF2 リスナーの初期化 (Cartographerの自己位置 map -> base_link 取得用)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _declare_parameters(self):
        self.declare_parameter('model_path', 'models/model')
        # LiDAR
        self.declare_parameter('lidar_num_beams', 216)
        self.declare_parameter('lidar_downsample_step', 5)
        self.declare_parameter('lidar_center_crop', True)
        self.declare_parameter('lidar_median_filter_size', 5)
        # Frame
        self.declare_parameter('frame_stack', 2)
        self.declare_parameter('frame_skip', 4)
        # Sim-to-Real / 物理限界
        self.declare_parameter('use_sim_to_real', True)
        self.declare_parameter('lidar_noise_std', 0.02)
        self.declare_parameter('steer_smoothing', 0.3)
        self.declare_parameter('speed_smoothing', 0.4)
        self.declare_parameter('max_steer_change_rate', 0.15)
        self.declare_parameter('max_speed_change_rate', 0.2)
        self.declare_parameter('speed_deadband', 0.05)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('steer_limit', 0.4189)
        self.declare_parameter('model_inference_timeout', 0.020)
        self.declare_parameter('use_pure_pursuit', False)
        # ゲイン
        self.declare_parameter('speed_multiplier', 1.0)
        self.declare_parameter('steer_multiplier', 1.0)
        # 安全
        self.declare_parameter('safety_enable', True)
        self.declare_parameter('safety_stop_dist', 0.2)
        self.declare_parameter('safety_check_angle', 30.0)
        # 復帰
        self.declare_parameter('recovery_enabled', True)
        self.declare_parameter('recovery_stop_dist', 0.3)
        self.declare_parameter('recovery_reverse_speed', -0.5)
        self.declare_parameter('recovery_steer_magnitude', 0.4)
        self.declare_parameter('recovery_brake_duration', 1.0)
        self.declare_parameter('recovery_stop_duration', 1.0)
        self.declare_parameter('recovery_back_duration', 3.0)
        self.declare_parameter('trigger_limit', 5)
        # 残差強化学習
        self.declare_parameter('racing_line_path', '')
        self.declare_parameter('use_residual_rl', True)
        self.declare_parameter('residual_steer_scale', 0.2)
        self.declare_parameter('residual_speed_scale', 1.0)
        # 右折強制
        self.declare_parameter('force_right_turn', False)
        self.declare_parameter('force_right_steer', 0.3)

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
        self._odom_received = True
        self.speed = msg.twist.twist.linear.x
        self.steer = msg.twist.twist.angular.z

        # 位置・向きの追跡
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        # クォータニオンから yaw への変換
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def _update_pose_from_tf(self):
        """
        Cartographer SLAM が計算した最新の自己位置 (map -> base_link の TF) を取得して自己位置を更新する。
        TFが取得できない場合（SLAM未起動時など）は、デッドレコニング（自己積算オドメトリ）に自動フォールバックする。
        """
        # 前回の呼び出しからの経過時間 dt の計算
        now = self.get_clock().now()
        if not hasattr(self, '_last_pose_time'):
            self._last_pose_time = now
        dt = (now - self._last_pose_time).nanoseconds / 1e9
        self._last_pose_time = now
        
        if dt > 0.5:  # 大きな遅延や初回起動時は積算をスキップ
            dt = 0.0

        try:
            # タイムアウト(10ms)を設けてTFを検索
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
            self.pose_x = t.transform.translation.x
            self.pose_y = t.transform.translation.y
            
            # クォータニオンから yaw への変換
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.pose_yaw = np.arctan2(siny_cosp, cosy_cosp)

            # 50回に1回、自己位置をログ出力
            if not hasattr(self, '_pose_log_cnt'):
                self._pose_log_cnt = 0
            self._pose_log_cnt += 1
            if self._pose_log_cnt % 50 == 0:
                self.get_logger().info(f"[SLAM Pose] x={self.pose_x:.3f}, y={self.pose_y:.3f}, yaw={self.pose_yaw:.3f}")
        except Exception as e:
            # TFが取得できない場合、アッカーマン・キネマティックモデルに基づくデッドレコニング（疑似オドメトリ）で積算
            wheelbase = 0.3255
            # self.steer と self.speed を用いて車両の位置と向きを積算
            self.pose_yaw += (self.speed / wheelbase) * np.tan(self.steer) * dt
            self.pose_x += self.speed * np.cos(self.pose_yaw) * dt
            self.pose_y += self.speed * np.sin(self.pose_yaw) * dt

            # 50回に1回、デッドレコニングでの動作状況を出力
            if not hasattr(self, '_tf_err_cnt'):
                self._tf_err_cnt = 0
            self._tf_err_cnt += 1
            if self._tf_err_cnt % 50 == 0:
                self.get_logger().info(
                    f"[Dead Reckoning Pose] (SLAM active: False) x={self.pose_x:.3f}, y={self.pose_y:.3f}, yaw={self.pose_yaw:.3f}"
                )

    def scan_callback(self, msg):
        # TF から最新の自己位置を更新
        self._update_pose_from_tf()

        # 1. LiDAR 前処理（ダウンサンプリング、中心クロップ、ノイズ除去を行うが、正規化は行わない生の距離 [m]）
        lidar = self.processor.process(msg.ranges, msg.range_max)

        # 2. 状態更新 (229次元を組み立ててバッファへ格納し、458次元状態を構築)
        self._update_state(lidar)

        # 3. 安全レイヤー / 推論 / Pure Pursuit 制御
        use_pure_pursuit = self.get_parameter('use_pure_pursuit').value
        self.safety_manager.collision_check_angle_deg = self.get_parameter('safety_check_angle').value
        self.safety_manager.collision_stop_dist = self.get_parameter('recovery_stop_dist').value
        self.safety_manager.collision_total_fov_deg = 200.0  # LiDAR FOV に合わせた値

        is_collision, dist = self.safety_manager.check_front_collision(lidar)

        # 緊急停止
        if self.get_parameter('safety_enable').value and dist < self.get_parameter('safety_stop_dist').value:
            self.get_logger().warn(f"EMERGENCY STOP! Dist: {dist:.2f}m")
            self.publish_drive(0.0, 0.0)
            return

        pure_pursuit_action = self._get_pure_pursuit_action()
        model_action = None
        model_healthy = True if use_pure_pursuit else False
        inference_errors = {}

        if not use_pure_pursuit and self.current_state is not None and self.model_manager.model_type is not None:
            self.model_health_monitor.start_inference()
            candidate_action = self.model_manager.predict(self.current_state)
            if candidate_action is not None:
                model_action = candidate_action
                model_healthy, inference_errors = self.model_health_monitor.end_inference(candidate_action)
            else:
                model_healthy = False
                inference_errors['prediction_failed'] = True

        current_time = time.time()
        self.safety_manager.update_state(
            model_healthy=model_healthy,
            collision=is_collision,
            x=self.pose_x,
            y=self.pose_y,
            yaw=self.pose_yaw,
            current_time=current_time
        )

        action = self.safety_manager.get_action(
            model_action=model_action,
            pure_pursuit_action=pure_pursuit_action,
            robot_state=(self.pose_x, self.pose_y, self.pose_yaw, self.speed),
            x=self.pose_x,
            y=self.pose_y,
            yaw=self.pose_yaw,
            current_time=current_time
        )
        self._apply_action(action)

    def _update_state(self, lidar):
        """
        学習環境 f1_env.py の _get_obs() と同一の 229次元 × 2スタック (458次元) 観測空間を再現。
        """
        # 1. LiDAR正規化 (0-1反転)
        lidar_norm = LidarProcessor.normalize(lidar, 30.0)
        norm_parts = [lidar_norm]

        # 2. 車両状態 [vel, steer]
        max_speed = self.get_parameter('max_speed').value
        steer_limit = self.get_parameter('steer_limit').value
        vel_norm = self.speed / max_speed
        steer_norm = self.steer / steer_limit
        norm_parts.append(np.array([vel_norm, steer_norm], dtype=np.float32))

        # 3. extra特徴量 [front_feat, min_feat, lr_asymmetry]
        extra_feats = LidarProcessor.compute_extra_features(lidar, 30.0)
        norm_parts.append(extra_feats)

        # 4. レーシングライン特徴
        rl_feat = self.racing_line.get_features(self.pose_x, self.pose_y, self.pose_yaw)
        norm_parts.append(rl_feat)

        # 5. 行動履歴
        norm_parts.append(self.prev_action)

        # 6. 残差RL用ベース制御器 (Pure Pursuit)
        min_speed = self.get_parameter('min_speed').value
        base_steer, base_speed = self.pp_controller.get_base_action(
            self.pose_x, self.pose_y, self.pose_yaw, self.speed, max_speed, min_speed
        )
        base_steer_norm = np.clip(base_steer / steer_limit, -1.0, 1.0)
        base_speed_norm = (base_speed - min_speed) / (max_speed - min_speed) * 2.0 - 1.0
        norm_parts.append(np.array([base_steer_norm, base_speed_norm], dtype=np.float32))

        # 現在の1フレーム特徴量 (229次元)
        current_obs = np.concatenate(norm_parts).astype(np.float32)

        # 履歴バッファへの追加
        self.obs_buffer.append(current_obs)

        # FRAME_SKIP=4 を考慮した FRAME_STACK=2 の積層処理
        stacked_obs = []
        for i in range(self.frame_stack_size):
            idx = -(i * self.frame_skip + 1)
            if abs(idx) > len(self.obs_buffer):
                stacked_obs.append(self.obs_buffer[0])
            else:
                stacked_obs.append(self.obs_buffer[idx])

        self.current_state = np.concatenate(stacked_obs)
        self.current_state = np.nan_to_num(self.current_state, nan=0.0, posinf=1.0, neginf=-1.0)

    def _get_pure_pursuit_action(self):
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        base_steer, base_speed = self.pp_controller.get_base_action(
            self.pose_x, self.pose_y, self.pose_yaw, self.speed, max_speed, min_speed
        )
        return np.array([base_steer, base_speed], dtype=np.float32)

    def _apply_action(self, action):
        pred_steer_res = float(action[0])
        pred_speed_res = float(action[1]) if len(action) >= 2 else 0.0

        # 残差スケールの取得
        steer_res_scale = self.get_parameter('residual_steer_scale').value
        speed_res_scale = self.get_parameter('residual_speed_scale').value

        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        steer_limit = self.get_parameter('steer_limit').value

        # 復旧/残差RL用ベース動作
        base_steer, base_speed = self.pp_controller.get_base_action(
            self.pose_x, self.pose_y, self.pose_yaw, self.speed, max_speed, min_speed
        )

        if self.get_parameter('use_residual_rl').value:
            # ベース行動 + 残差補正
            final_steer = base_steer + pred_steer_res * steer_res_scale
            final_speed = base_speed + pred_speed_res * speed_res_scale
        else:
            # 通常の直接制御 (フォールバック)
            final_steer = pred_steer_res * steer_limit
            final_speed = min_speed + (pred_speed_res + 1.0) * (max_speed - min_speed) / 2.0

        # クリップ
        final_steer = np.clip(final_steer, -steer_limit, steer_limit)
        final_speed = np.clip(final_speed, min_speed, max_speed)

        # ゲインの適用
        final_speed *= self.get_parameter('speed_multiplier').value
        final_steer *= self.get_parameter('steer_multiplier').value

        # Sim-to-Real 処理
        if self.get_parameter('use_sim_to_real').value:
            # デッドバンド
            if abs(final_speed) < self.get_parameter('speed_deadband').value:
                final_speed = 0.0

            # スルーレート & EMA
            max_s_diff = self.get_parameter('max_steer_change_rate').value
            max_v_diff = self.get_parameter('max_speed_change_rate').value
            s_alpha = self.get_parameter('steer_smoothing').value
            v_alpha = self.get_parameter('speed_smoothing').value

            s_clipped = self.last_steer + np.clip(final_steer - self.last_steer, -max_s_diff, max_s_diff)
            v_clipped = self.last_speed + np.clip(final_speed - self.last_speed, -max_v_diff, max_v_diff)

            final_steer = s_alpha * s_clipped + (1.0 - s_alpha) * self.last_steer
            final_speed = v_alpha * v_clipped + (1.0 - v_alpha) * self.last_speed

        # 右折のみを強制
        if self.get_parameter('force_right_turn').value:
            right_steer = float(self.get_parameter('force_right_steer').value)
            final_steer = -max(abs(final_steer), abs(right_steer))

        # 次ステップ用の行動履歴の保存 (正規化スケール)
        self.prev_action = np.array([pred_steer_res, pred_speed_res], dtype=np.float32)

        # /odom トピックが無い場合（実機のみのSLAM環境など）の車速フィードバックの代用
        if not hasattr(self, '_odom_received') or not self._odom_received:
            self.speed = final_speed

        self.last_steer, self.last_speed = final_steer, final_speed
        self.publish_drive(final_speed, final_steer)

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
