#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time

class SafetyTest(Node):
    def __init__(self):
        super().__init__('safety_test_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.subscriber = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
        self.test_success = False
        self.received_any = False

        self.get_logger().info("Safety Test Node Started. Running...")

    def drive_callback(self, msg):
        self.received_any = True
        speed = msg.drive.speed
        if speed == 0.0:
            self.get_logger().info(f"RECEIVED speed=0.0! Safety layer is working.")
            self.test_success = True
        else:
            self.get_logger().info(f"Received speed={speed:.3f}. Waiting for emergency command...")

    def run_test(self):
        # Publish a 'Crash' scan: obstacle at 0.15m (threshold is 0.3m)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = [0.15] * 1080
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SafetyTest()
    
    node.get_logger().info("Starting test loop: publishing 0.15m scans every 0.1s...")
    
    # Run loop for 5 seconds or until success
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 10.0:
        node.run_test()
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.test_success:
            break
        # スピードを上げて高頻度でパージ
        # time.sleep(0.1) # loop speed is handled by spin_once timeout

    if node.test_success:
        print("\n=== FINAL RESULT: SAFETY TEST PASSED ===\n")
    elif not node.received_any:
        print("\n=== FINAL RESULT: NO DRIVE MESSAGE RECEIVED (Check if rl_driver is running) ===\n")
    else:
        print("\n=== FINAL RESULT: SAFETY TEST FAILED ===\n")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
