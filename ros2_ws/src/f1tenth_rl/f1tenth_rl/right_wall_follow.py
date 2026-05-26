import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class RightWallFollow(Node):
    """LiDAR 右側距離を使って右壁追従を行うシンプルな ROS2 ノード。"""

    def __init__(self):
        super().__init__('right_wall_follow')

        # 制御パラメータ
        self.target_distance = self.declare_parameter('target_distance', 0.7).value
        self.kp = self.declare_parameter('kp', 1.0).value
        self.max_steer = self.declare_parameter('max_steer', 1.2).value
        self.max_speed = self.declare_parameter('max_speed', 0.7).value
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.max_range = self.declare_parameter('max_range', 30.0).value
        self.right_sector_min_deg = self.declare_parameter('right_sector_min_deg', -120.0).value
        self.right_sector_max_deg = self.declare_parameter('right_sector_max_deg', -60.0).value
        self.speed_reduction_on_error = self.declare_parameter('speed_reduction_on_error', 0.1).value

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('RightWallFollow node started.')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        count = len(ranges)
        if count == 0 or msg.angle_increment == 0.0:
            self.get_logger().warn('Invalid LaserScan input')
            return

        angles = msg.angle_min + np.arange(count, dtype=np.float32) * msg.angle_increment
        min_rad = math.radians(self.right_sector_min_deg)
        max_rad = math.radians(self.right_sector_max_deg)
        sector_mask = (angles >= min_rad) & (angles <= max_rad)

        if not np.any(sector_mask):
            self.get_logger().warn('Right sector is empty for this LaserScan configuration')
            return

        right_ranges = ranges[sector_mask]
        right_ranges = right_ranges[np.isfinite(right_ranges)]
        if right_ranges.size == 0:
            self.get_logger().warn('No valid right-side LiDAR returns')
            return

        right_distance = float(np.median(right_ranges))
        error = right_distance - self.target_distance

        # 右側距離が大きいほど右折、近いほど左折
        steer = -self.kp * error
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        # 角度誤差が大きいときは速度を落とす
        speed = float(self.max_speed * (1.0 - min(abs(error) / self.target_distance, 1.0) * self.speed_reduction_on_error))
        speed = float(np.clip(speed, self.min_speed, self.max_speed))

        self.publish_drive(speed, steer)

    def publish_drive(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('RightWallFollow node shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RightWallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
