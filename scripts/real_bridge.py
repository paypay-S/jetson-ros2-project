import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
import math
import time

class RealBridge(Node):
    def __init__(self):
        super().__init__('real_bridge')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # サブスクライバ: /cmd_vel受信
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # パブリッシャ: /drive (実機) と /odom (SLAM用)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TFブロードキャスター
        self.tf_broadcaster = TransformBroadcaster(self)

        # 更新タイマー (50Hz)
        self.create_timer(0.02, self.update)
        
        self.get_logger().info('Real Bridge (Twist -> Ackermann + Pseudo Odom/TF) started.')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. 位置の積分
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # 2. Odometry メッセージの作成と発行
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # 3. TF の発行 (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # base_link -> laser
        t_laser = TransformStamped()
        t_laser.header.stamp = now.to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.11
        t_laser.transform.translation.z = 0.12
        t_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t, t_laser])

        # 4. /drive 命令に変換
        drive = AckermannDriveStamped()
        drive.header.stamp = now.to_msg()
        drive.drive.speed = -self.v
        drive.drive.steering_angle = self.w
        self.drive_pub.publish(drive)

def main(args=None):
    rclpy.init(args=args)
    node = RealBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
