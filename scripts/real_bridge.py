import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
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

        # サブスクライバ: /cmd_vel (キーボード) を受信
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # パブリッシャ: /drive (実機) を送信
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # TFブロードキャスター: 疑似オドメトリ (odom -> base_link) を送信
        self.tf_broadcaster = TransformBroadcaster(self)

        # 更新タイマー (50Hz)
        self.create_timer(0.02, self.update)
        
        self.get_logger().info('Real Bridge (Twist -> Ackermann + Pseudo TF) started.')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. 位置の積分 (疑似オドメトリの計算)
        # 実際には車が動いていなくても、命令に従ってSLAM上の位置を動かす
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # 2. TF を一括発行
        # odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # base_link -> laser (静的変換を動的に発行して時刻同期を保証する)
        t_laser = TransformStamped()
        t_laser.header.stamp = now.to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.11
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.12
        t_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t, t_laser])

        # 3. /drive 命令に変換して送信 (実機の挙動に合わせて符号を調整)
        drive = AckermannDriveStamped()
        drive.header.stamp = now.to_msg()
        drive.header.frame_id = 'base_link'
        
        # 前後が逆なのを補正 (-self.v)、ステアリングは正転に修正 (self.w)
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
