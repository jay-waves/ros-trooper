#!/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import time

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_publisher')
        self._logger = self.get_logger()
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._logger.info(f'Waiting for action server to activate...')
        self._action_client.wait_for_server()

    def send_goal(self):
        msg = self.get_random_pose()
        self._logger.info(f'Moving to goal...')
        goal_future = self._action_client.send_goal_async(msg)
        goal_future.add_done_callback(self.on_response)

    def on_response(self, future):
        handler = future.result()
        if not handler.accepted:
            self.get_logger().info("Goal rejected")
            time.sleep(0.5)
            self.send_goal()
        else:
            self._logger.info("Goal accepted")
            result_future = handler.get_result_async()
            result_future.add_done_callback(self.on_finish)

    def on_finish(self, future):
        # begin next goal
        result = future.result().result
        self._logger.info("Finished, planning next goal...")
        self.send_goal()

    def get_random_pose(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = random.uniform(-3,3)
        goal_msg.pose.pose.position.y = random.uniform(-3,3)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = -0.37
        goal_msg.pose.pose.orientation.w = 1.0
        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    client = RandomPosePublisher()
    client.send_goal()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
