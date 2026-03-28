import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # Publishers / Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.create_timer(0.05, self.update_state)  # 20 Hz
        self.create_timer(0.1, self.publish_scan)   # 10 Hz

        # Map environment: 10m x 10m room
        self.room_size = 10.0

        self.get_logger().info('Mock robot initialized. Ready for teleop!')

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

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
    rclpy.shutdown()

if __name__ == '__main__':
    main()
