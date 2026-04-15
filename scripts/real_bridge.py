import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

class RealBridge(Node):
    def __init__(self):
        super().__init__('real_bridge')
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # サブスクライバ: /cmd_vel受信
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # パブリッシャ: /drive (実機)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # TFブロードキャスター
        self.tf_broadcaster = TransformBroadcaster(self)

        # 更新タイマー (50Hz)
        self.create_timer(0.02, self.update)
        
        self.get_logger().info('Real Bridge (Twist -> Ackermann + TF) started.')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        self.last_time = now

        # TF の発行 (base_link -> laser)
        t_laser = TransformStamped()
        t_laser.header.stamp = now.to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.11
        t_laser.transform.translation.z = 0.12
        t_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t_laser])

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
