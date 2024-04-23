#!/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import random

class VelPub(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.pub_vel()

    def pub_vel(self):
        msg = Twist()
        msg.linear.x = random.gauss(0.2, 0.1)
        msg.linear.x = 0.7
        msg.angular.z = random.gauss(0, 0.02)

        self.get_logger().info('Publishing velocity')
        while self.pub.get_subscription_count() == 0:
            time.sleep(0.5)
        self.pub.publish(msg)

        # quit after pulishing pose
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    VelPub()

if __name__ == '__main__':
    main()

