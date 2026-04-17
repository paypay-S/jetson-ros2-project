#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import subprocess
import os
import threading
import time
from datetime import datetime

msg = """
F1TENTH Teleop & Map Manager (Robust Edition)
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Map Saving & Reset:
1 ~ 9 : Save map to 'maps/<session>/map_<num>_.../' 
        and RESET SLAM to start a new map.

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

# 保存中フラグ
is_saving = False
save_lock = threading.Lock()

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_map_sequence(node, num):
    global is_saving
    
    try:
        timestamp = datetime.now().strftime("%m%d_%H%M%S")
        map_name = f"map_{num}_{timestamp}"
        session_id = os.environ.get('SESSION_ID', 'default_session')
        maps_dir = os.path.join(os.environ.get('HOME', '/home/toyonishiorin'), 'projects/f1tenth-project/maps', session_id)
        
        # フォルダ分け
        save_path = os.path.join(maps_dir, map_name)
        os.makedirs(save_path, exist_ok=True)
        
        pbstream_path = os.path.join(save_path, f"{map_name}.pbstream")
        map_yaml_path = os.path.join(save_path, map_name)

        node.get_logger().info(f"\n[MANAGER] STARTING SAVE SEQUENCE for Map {num}...")
        node.get_logger().info(f"[MANAGER] Saving into: {save_path}")

        # 1. Save Cartographer State (.pbstream)
        node.get_logger().info("1/3: Saving .pbstream (timeout: 20s)...")
        pb_cmd = [
            "ros2", "service", "call", "/write_state", 
            "cartographer_ros_msgs/srv/WriteState", 
            f"{{filename: '{pbstream_path}'}}"
        ]
        try:
            result_pb = subprocess.run(pb_cmd, capture_output=True, timeout=20.0)
            if result_pb.returncode == 0:
                node.get_logger().info("Successfully saved .pbstream")
            else:
                node.get_logger().error(f"Failed to save .pbstream: {result_pb.stderr.decode()}")
        except subprocess.TimeoutExpired:
            node.get_logger().error("pbstream save TIMEOUT expired.")

        # 2. Save Map Image (.pgm / .yaml)
        node.get_logger().info("2/3: Saving .pgm and .yaml (timeout: 45s)...")
        map_cmd = [
            "ros2", "run", "nav2_map_server", "map_saver_cli", 
            "-f", map_yaml_path, 
            "--ros-args", "-p", "save_map_timeout:=10000.0"
        ]
        try:
            result_map = subprocess.run(map_cmd, capture_output=True, timeout=45.0)
            if result_map.returncode == 0:
                node.get_logger().info("Successfully saved map images (.pgm/.yaml)")
            else:
                node.get_logger().error(f"Failed to save map images: {result_map.stderr.decode()}")
        except subprocess.TimeoutExpired:
            node.get_logger().error("map_saver_cli TIMEOUT expired. Map image might be incomplete.")

        # 3. Reset SLAM (Always try to reset even if save failed)
        node.get_logger().info("3/3: Resetting SLAM nodes...")
        subprocess.run("pkill -9 -f cartographer_node", shell=True)
        subprocess.run("pkill -9 -f occupancy_grid_node", shell=True)
        
        node.get_logger().info("\033[1;32m[MANAGER] DONE! Map processing finished successfully.\033[0m")
        node.get_logger().info("-" * 40)

    except Exception as e:
        node.get_logger().error(f"\033[1;31mUnexpected error in save_map_sequence: {e}\033[0m")

    finally:
        with save_lock:
            is_saving = False
            print("\n" + "="*50)
            print("\033[1;32m   [READY] SLAM RESET COMPLETE! START MAPPING NOW!   \033[0m")
            print("="*50 + "\n")
            print("Operation: WASD/Arrows to drive, 1-9 to save again.")

def start_save_thread(node, num):
    global is_saving
    with save_lock:
        if is_saving:
            node.get_logger().warn("Save already in progress. Ignoring request.")
            return
        is_saving = True
    
    thread = threading.Thread(target=save_map_sequence, args=(node, num))
    thread.start()

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = Node('teleop_map_manager')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            # Map Saving Keys
            if key in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
                start_save_thread(node, key)
                continue

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(f"currently:\tspeed {speed}\tturn {turn}")
            elif key == 'k':
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
            else:
                if (key == '\x03'): # CTRL-C
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
