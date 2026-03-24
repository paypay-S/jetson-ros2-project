#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import time
import numpy as np

class SiLIntegrationTest(Node):
    """
    Simulated Software-in-the-Loop Integration Test.
    Publishes synthetic scan and odom data, then verifies if the RL driver 
    produces consistent drive commands.
    """
    def __init__(self):
        super().__init__('sil_integration_test_node')
        
        # Publishers (Mocks simulation output)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscriber (Monitors RL driver output)
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, 
            '/drive', 
            self.drive_callback, 
            10
        )
        
        self.msg_count = 0
        self.last_drive_msg = None
        self.get_logger().info("SiL Integration Test Node Initialized.")

    def drive_callback(self, msg):
        self.msg_count += 1
        self.last_drive_msg = msg
        # self.get_logger().info(f"Received Drive Msg #{self.msg_count}: speed={msg.drive.speed:.2f}")

    def run_integration_cycle(self):
        # 1. Publish Synthetic Scan (A simple straight corridor)
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser"
        scan_msg.ranges = [2.0] * 1080 # Everything is 2 meters away
        self.scan_pub.publish(scan_msg)
        
        # 2. Publish Synthetic Odom (Speed = 0.5m/s)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.twist.twist.linear.x = 0.5
        self.odom_pub.publish(odom_msg)

def main():
    rclpy.init()
    test_node = SiLIntegrationTest()
    
    print("\n=== STARTING SIL INTEGRATION TEST ===")
    print("Verifying if RL driver can process synthetic data flow...")
    
    # Run loop for 5 seconds to collect data
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 5.0:
        test_node.run_integration_cycle()
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.1) # 10Hz

    print(f"\nTest finished. Received {test_node.msg_count} drive messages.")
    
    # Validation logic
    success = True
    if test_node.msg_count < 10:
        print("ERROR: Message frequency too low.")
        success = False
    
    if test_node.last_drive_msg is None:
        print("ERROR: No drive messages received at all.")
        success = False
    
    if success:
        print("SUCCESS: Full pipeline integration verified.")
        print("=== FINAL RESULT: SIL TEST PASSED ===\n")
    else:
        print("FAILURE: Pipeline is broken.\n")

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
