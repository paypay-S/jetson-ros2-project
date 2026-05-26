import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import os
import csv

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        # パラメータの宣言 (レーシングラインの初期位置ロード用)
        self.declare_parameter('racing_line_path', '')
        rl_path = self.get_parameter('racing_line_path').value

        # Robot State (Default)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # レーシングラインがある場合、最初のウェイポイントを初期位置に自動セット
        if rl_path and os.path.exists(rl_path):
            try:
                with open(rl_path, 'r') as f:
                    reader = csv.reader(f)
                    rows = list(reader)
                    # ヘッダーを避けて最初の有効な点を探索
                    for row in rows:
                        try:
                            self.x = float(row[0])
                            self.y = float(row[1])
                            # 3列目がyaw角（向き）の場合、それを適用
                            if len(row) > 2:
                                self.theta = float(row[2])
                            break
                        except ValueError:
                            continue
                self.get_logger().info(f"Initialized mock robot spawn pose at racing line start: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")
            except Exception as e:
                self.get_logger().warn(f"Could not load racing line for initialization: {str(e)}")

        # Publishers / Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.create_timer(0.05, self.update_state)  # 20 Hz
        self.create_timer(0.1, self.publish_scan)   # 10 Hz

        # Map environment: 50m x 50m room (十分大きな仮想部屋)
        self.room_size = 50.0

        self.get_logger().info('Mock robot initialized. Ready for simulation and teleop!')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def drive_cb(self, msg: AckermannDriveStamped):
        # アッカーマンキネマティクスに基づく旋回角速度 w の計算
        # w = v * tan(delta) / L (wheelbase)
        self.v = msg.drive.speed
        wheelbase = 0.3255
        if abs(msg.drive.steering_angle) > 1e-5:
            self.w = (self.v / wheelbase) * math.tan(msg.drive.steering_angle)
        else:
            self.w = 0.0

    def update_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Update pose
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        # Publish fake Odom (slam_toolbox requires robust TF mostly, but odom is good too)
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        self.odom_pub.publish(odom)

    def publish_scan(self):
        # Generate fake 1080 beams for full 360 deg
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 1080
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = 20.0

        ranges = []
        for i in range(1080):
            angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.theta + angle
            
            # Distance to the wall
            dists = []
            dx = math.cos(global_angle)
            dy = math.sin(global_angle)
            
            # Intersection with x = -5, 5
            if dx > 1e-5:
                # intersect with x = 5 (right wall)
                dw = (self.room_size/2.0 - self.x) / dx
                if dw > 0: dists.append(dw)
            elif dx < -1e-5:
                dw = (-self.room_size/2.0 - self.x) / dx
                if dw > 0: dists.append(dw)
                
            # Intersection with y = -5, 5
            if dy > 1e-5:
                # intersect with y = 5 (top wall)
                dh = (self.room_size/2.0 - self.y) / dy
                if dh > 0: dists.append(dh)
            elif dy < -1e-5:
                dh = (-self.room_size/2.0 - self.y) / dy
                if dh > 0: dists.append(dh)
            
            r = min(dists) if dists else 20.0
            
            # Add some Gaussian noise
            r += np.random.normal(0, 0.02)
            ranges.append(float(r))
            
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
