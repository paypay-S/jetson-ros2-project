import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO
import numpy as np


class RLDriver(Node):

    def __init__(self):
        super().__init__("rl_driver")

        self.model = PPO.load("model.zip")

        self.speed = 0.0
        self.steer = 0.0

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):

        lidar = np.array(msg.ranges)

        state = np.concatenate([
            lidar,
            np.array([self.speed, self.steer])
        ])

        action, _ = self.model.predict(state)

        print("action:", action)