"""
hardware_bridge.py

/drive (AckermannDriveStamped) を受け取り、
PCA9685経由でステアリングサーボとESCを制御するROS2ノード。

チャンネル割り当て:
  ch0: ステアリングサーボ (center=4700, left=3700, right=5700)
  ch1: ESC/モーター      (stop=5200, forward=5800, reverse=4000)
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import math
import time

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    HW_AVAILABLE = True
except Exception:
    HW_AVAILABLE = False


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def map_range(value, in_min, in_max, out_min, out_max):
    """線形マッピング"""
    if in_max == in_min:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    ratio = clamp(ratio, 0.0, 1.0)
    return int(out_min + ratio * (out_max - out_min))


class HardwareBridge(Node):

    def __init__(self):
        super().__init__('hardware_bridge')

        # ─── パラメータ宣言 ───────────────────────────────────────────
        # ステアリング (ch0)
        self.declare_parameter('steer_ch', 0)
        self.declare_parameter('steer_center', 4700)
        self.declare_parameter('steer_left',   3700)   # steering_angle > 0 (左)
        self.declare_parameter('steer_right',  5700)   # steering_angle < 0 (右)
        self.declare_parameter('steer_max_angle', 0.4)  # rad, モデル出力の最大ステア角
        self.declare_parameter('steer_bias', 0)         # ステアリングのセンターオフセット調整 (duty_cycle単位)

        # ESC (ch1)
        self.declare_parameter('esc_ch', 1)
        self.declare_parameter('esc_stop',    5200)
        self.declare_parameter('esc_forward', 5800)
        self.declare_parameter('esc_reverse', 4000)

        # 固定走行速度モード
        self.declare_parameter('fixed_speed_mode', True)
        self.declare_parameter('fixed_esc_duty', 5800)   # 前進時のduty_cycle
        self.declare_parameter('speed_threshold', 0.05)  # これ以上のspeedで前進

        # ESCアーム待機時間 (秒)
        self.declare_parameter('esc_arm_duration', 3.0)

        # ─── パラメータ取得 ───────────────────────────────────────────
        self.steer_ch      = self.get_parameter('steer_ch').value
        self.steer_center  = self.get_parameter('steer_center').value
        self.steer_left    = self.get_parameter('steer_left').value
        self.steer_right   = self.get_parameter('steer_right').value
        self.steer_max_rad = self.get_parameter('steer_max_angle').value
        self.steer_bias    = self.get_parameter('steer_bias').value

        self.esc_ch      = self.get_parameter('esc_ch').value
        self.esc_stop    = self.get_parameter('esc_stop').value
        self.esc_forward = self.get_parameter('esc_forward').value
        self.esc_reverse = self.get_parameter('esc_reverse').value

        self.fixed_speed_mode = self.get_parameter('fixed_speed_mode').value
        self.fixed_esc_duty   = self.get_parameter('fixed_esc_duty').value
        self.speed_threshold  = self.get_parameter('speed_threshold').value
        self.esc_arm_duration = self.get_parameter('esc_arm_duration').value

        # ─── PCA9685 初期化 ──────────────────────────────────────────
        self.pca = None
        if HW_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=0x40)
                self.pca.frequency = 50
                self.get_logger().info('PCA9685 initialized')

                # ESCアーム: STOPを送って待機
                self.get_logger().info(
                    f'ESC arming: sending STOP ({self.esc_stop}) for {self.esc_arm_duration}s ...'
                )
                self.pca.channels[self.steer_ch].duty_cycle = self.steer_center
                self.pca.channels[self.esc_ch].duty_cycle = self.esc_stop
                time.sleep(self.esc_arm_duration)
                self.get_logger().info('ESC armed. Ready to drive.')

            except Exception as e:
                self.get_logger().error(f'Failed to initialize PCA9685: {e}')
                self.pca = None
        else:
            self.get_logger().warn(
                'adafruit_pca9685 not available. Running in DRY-RUN mode (no hardware output).'
            )

        # ─── サブスクライバ ──────────────────────────────────────────
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )
        self.get_logger().info('hardware_bridge node started. Subscribing to /drive ...')

    # ─── コールバック ────────────────────────────────────────────────
    def drive_callback(self, msg):
        speed        = msg.drive.speed
        steer_angle  = msg.drive.steering_angle  # rad, 正=左, 負=右

        # ── ステアリング変換 ──
        # steer_angle: [-steer_max_rad, +steer_max_rad] → [steer_right, steer_left]
        steer_duty = map_range(
            steer_angle,
            -self.steer_max_rad,  # 右最大
             self.steer_max_rad,  # 左最大
             self.steer_right,
             self.steer_left
        )

        # ── ステアリングバイアスの適用 (実機の個体差を吸収) ──
        steer_duty += self.steer_bias
        steer_duty = clamp(steer_duty, min(self.steer_left, self.steer_right), max(self.steer_left, self.steer_right))

        # ── ESC (速度) 変換 ──
        if self.fixed_speed_mode:
            if speed > self.speed_threshold:
                esc_duty = self.fixed_esc_duty
            elif speed < -self.speed_threshold:
                esc_duty = self.esc_reverse
            else:
                esc_duty = self.esc_stop
        else:
            # 将来の拡張: speed (m/s) をduty_cycleにスケール
            if speed >= 0:
                esc_duty = map_range(speed, 0.0, 3.0, self.esc_stop, self.esc_forward)
            else:
                esc_duty = map_range(-speed, 0.0, 3.0, self.esc_stop, self.esc_reverse)

        self.get_logger().debug(
            f'drive: speed={speed:.2f} steer={steer_angle:.3f}rad '
            f'→ steer_duty={steer_duty} esc_duty={esc_duty}'
        )

        self._set_hardware(steer_duty, esc_duty)

    def _set_hardware(self, steer_duty: int, esc_duty: int):
        if self.pca is not None:
            self.pca.channels[self.steer_ch].duty_cycle = steer_duty
            self.pca.channels[self.esc_ch].duty_cycle   = esc_duty
        else:
            # DRY-RUNモード: ログ出力のみ
            self.get_logger().info(
                f'[DRY-RUN] steer_ch={self.steer_ch} duty={steer_duty}, '
                f'esc_ch={self.esc_ch} duty={esc_duty}'
            )

    def destroy_node(self):
        """シャットダウン時にハードウェアを安全停止"""
        self.get_logger().info('Shutting down: sending STOP to ESC and centering steering.')
        if self.pca is not None:
            self.pca.channels[self.steer_ch].duty_cycle = self.steer_center
            self.pca.channels[self.esc_ch].duty_cycle   = self.esc_stop
            self.pca.deinit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
