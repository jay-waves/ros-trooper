#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.timer import Timer
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_publisher')
        self._logger = self.get_logger()
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._logger.info('Waiting for action server to activate...')
        self._action_client.wait_for_server()
        
        # 设置定时器，每5秒调用一次send_goal
        self.timer = self.create_timer(5.0, self.send_goal)

    def send_goal(self):
        msg = self.get_random_pose()
        self._logger.info('Moving to goal...')
        goal_future = self._action_client.send_goal_async(msg)
        goal_future.add_done_callback(self.on_response)

    def on_response(self, future):
        handler = future.result()
        if not handler.accepted:
            self.get_logger().info("Goal rejected")
        else:
            self._logger.info("Goal accepted")
            result_future = handler.get_result_async()
            result_future.add_done_callback(self.on_finish)

    def on_finish(self, future):
        result = future.result().result
        self._logger.info(f"Finished, status: {result}")

    def get_random_pose(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = random.uniform(-3, 3)
        goal_msg.pose.pose.position.y = random.uniform(-3, 3)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = -0.37
        goal_msg.pose.pose.orientation.w = 1.0
        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    client = RandomPosePublisher()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

