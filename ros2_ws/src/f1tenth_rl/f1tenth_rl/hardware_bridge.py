"""
hardware_bridge.py

/drive (AckermannDriveStamped) を受け取り、
PCA9685経由でステアリングサーボとESCを制御するROS2ノード。
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time
import sys
import os
from typing import Optional

# 仮想環境のライブラリパスを動的に追加 (WSL2/Jetson 共用)
def setup_venv_path():
    home_dir = os.path.expanduser('~')
    venv_path = os.path.join(home_dir, 'projects/f1tenth-project/jetson-ros2/lib/python3.10/site-packages')
    if os.path.exists(venv_path) and venv_path not in sys.path:
        sys.path.append(venv_path)

setup_venv_path()

# ハードウェアライブラリのインポート
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    HW_AVAILABLE = True
    IMPORT_ERROR_MSG = ""
except Exception as e:
    HW_AVAILABLE = False
    IMPORT_ERROR_MSG = str(e)


class DriveMapper:
    """
    Ackermann指令値をサーボ/ESCのPWMデューティサイクルに変換するクラス。
    """
    def __init__(self, 
                 steer_center: int, steer_left: int, steer_right: int, steer_max_rad: float, steer_bias: int, steer_flip: bool,
                 esc_stop: int, esc_forward: int, esc_reverse: int, speed_flip: bool,
                 fixed_speed_mode: bool, fixed_esc_duty: int, speed_threshold: float):
        self.steer_center = steer_center
        self.steer_left = steer_left
        self.steer_right = steer_right
        self.steer_max_rad = steer_max_rad
        self.steer_bias = steer_bias
        self.steer_flip = steer_flip
        
        self.esc_stop = esc_stop
        self.esc_forward = esc_forward
        self.esc_reverse = esc_reverse
        self.speed_flip = speed_flip
        
        self.fixed_speed_mode = fixed_speed_mode
        self.fixed_esc_duty = fixed_esc_duty
        self.speed_threshold = speed_threshold

    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    @staticmethod
    def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> int:
        if in_max == in_min:
            return int(out_min)
        ratio = (value - in_min) / (in_max - in_min)
        ratio = DriveMapper.clamp(ratio, 0.0, 1.0)
        return int(out_min + ratio * (out_max - out_min))

    def map_drive(self, speed: float, steer_angle: float) -> tuple[int, int]:
        """
        速度とステアリング角をPWMデューティに変換する。
        """
        # 反転の適用
        if self.speed_flip:
            speed = -speed
        if self.steer_flip:
            steer_angle = -steer_angle

        # ステアリング変換
        steer_duty = self.map_range(
            steer_angle,
            -self.steer_max_rad,  # 右最大
             self.steer_max_rad,  # 左最大
             self.steer_right,
             self.steer_left
        )
        steer_duty += self.steer_bias
        
        # クランプ (ステアリング)
        s_min, s_max = sorted([self.steer_left, self.steer_right])
        steer_duty = int(self.clamp(steer_duty, s_min, s_max))

        # ESC変換
        if self.fixed_speed_mode and speed > self.speed_threshold:
            esc_duty = self.fixed_esc_duty
        else:
            if speed >= 0:
                esc_duty = self.map_range(speed, 0.0, 1.0, self.esc_stop, self.esc_forward)
            else:
                esc_duty = self.map_range(-speed, 0.0, 1.0, self.esc_stop, self.esc_reverse)
        
        return steer_duty, esc_duty


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        self._declare_parameters()
        
        # 内部状態
        self.pca: Optional[PCA9685] = None
        self.mapper = self._create_mapper()
        
        self._init_hardware()
        
        # サブスクライバ
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/drive', self.drive_callback, 10
        )
        self.get_logger().info('hardware_bridge node started.')

    def _declare_parameters(self):
        self.declare_parameter('steer_ch', 0)
        self.declare_parameter('steer_center', 4700)
        self.declare_parameter('steer_left',   3700)
        self.declare_parameter('steer_right',  5700)
        self.declare_parameter('steer_max_angle', 0.4)
        self.declare_parameter('steer_bias', 0)
        self.declare_parameter('steer_flip', False)

        self.declare_parameter('esc_ch', 1)
        self.declare_parameter('esc_stop',    5200)
        self.declare_parameter('esc_forward', 5800)
        self.declare_parameter('esc_reverse', 4000)
        self.declare_parameter('speed_flip', False)

        self.declare_parameter('fixed_speed_mode', False)
        self.declare_parameter('fixed_esc_duty', 5800)
        self.declare_parameter('speed_threshold', 0.05)
        self.declare_parameter('esc_arm_duration', 3.0)

    def _create_mapper(self) -> DriveMapper:
        return DriveMapper(
            steer_center=self.get_parameter('steer_center').value,
            steer_left=self.get_parameter('steer_left').value,
            steer_right=self.get_parameter('steer_right').value,
            steer_max_rad=self.get_parameter('steer_max_angle').value,
            steer_bias=self.get_parameter('steer_bias').value,
            steer_flip=self.get_parameter('steer_flip').value,
            esc_stop=self.get_parameter('esc_stop').value,
            esc_forward=self.get_parameter('esc_forward').value,
            esc_reverse=self.get_parameter('esc_reverse').value,
            speed_flip=self.get_parameter('speed_flip').value,
            fixed_speed_mode=self.get_parameter('fixed_speed_mode').value,
            fixed_esc_duty=self.get_parameter('fixed_esc_duty').value,
            speed_threshold=self.get_parameter('speed_threshold').value
        )

    def _init_hardware(self):
        if not HW_AVAILABLE:
            self.get_logger().warn(f'Hardware not available (Dry-run): {IMPORT_ERROR_MSG}')
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=0x40)
            self.pca.frequency = 50
            
            # ESCアーム処理
            arm_duration = self.get_parameter('esc_arm_duration').value
            esc_stop = self.get_parameter('esc_stop').value
            steer_center = self.get_parameter('steer_center').value
            
            self.get_logger().info(f'ESC arming for {arm_duration}s...')
            self._set_duty(self.get_parameter('steer_ch').value, steer_center)
            self._set_duty(self.get_parameter('esc_ch').value, esc_stop)
            time.sleep(arm_duration)
            self.get_logger().info('ESC armed.')
        except Exception as e:
            self.get_logger().error(f'Failed to init PCA9685: {e}')
            self.pca = None

    def drive_callback(self, msg: AckermannDriveStamped):
        steer_duty, esc_duty = self.mapper.map_drive(msg.drive.speed, msg.drive.steering_angle)
        
        self.get_logger().debug(f'speed={msg.drive.speed:.2f} steer={msg.drive.steering_angle:.3f} -> s={steer_duty} e={esc_duty}')
        
        self._set_duty(self.get_parameter('steer_ch').value, steer_duty)
        self._set_duty(self.get_parameter('esc_ch').value, esc_duty)

    def _set_duty(self, channel: int, duty: int):
        if self.pca is not None:
            self.pca.channels[channel].duty_cycle = int(duty)
        else:
            self.get_logger().info(f'[DRY-RUN] ch={channel} duty={duty}')

    def destroy_node(self):
        self.get_logger().info('Shutting down: stopping hardware.')
        if self.pca is not None:
            try:
                self._set_duty(self.get_parameter('steer_ch').value, self.get_parameter('steer_center').value)
                self._set_duty(self.get_parameter('esc_ch').value, self.get_parameter('esc_stop').value)
                self.pca.deinit()
            except Exception:
                pass
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
