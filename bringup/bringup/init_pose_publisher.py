from colorama import init
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPosePublisher(Node):
    '''send initial pose to nav2_amcl'''
    # msg:
    msg_cnt = 6
    # header stamp
    sec = 5
    nanosec = 0
    # header fram_id
    frame_id = 'map'
    # pose position
    pos_x= -1.9906362295150757 
    pos_y= -0.5007724761962891
    pos_z= 0.0
    # pose orientation
    ori_x = 0.0
    ori_y = 0.0
    ori_z = 0.009562732387379319
    ori_w = 0.999954276029303
    # pose covariance
    covariance= [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]

    def __init__(self):
        super().__init__('init_pose_publisher')
        
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.shutdown_flag = False
        self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        nodes_info = self.get_node_names()
        if 'amcl' in nodes_info:
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.stamp.sec = self.sec
            init_pose.header.stamp.nanosec = self.nanosec
            init_pose.header.frame_id = self.frame_id
            init_pose.pose.pose.position.x = self.pos_x
            init_pose.pose.pose.position.y = self.pos_y
            init_pose.pose.pose.position.z = self.pos_z
            init_pose.pose.pose.orientation.x = self.ori_x
            init_pose.pose.pose.orientation.y = self.ori_y
            init_pose.pose.pose.orientation.z = self.ori_z
            init_pose.pose.pose.orientation.w = self.ori_w
            init_pose.pose.covariance = self.covariance
            self.publisher.publish(init_pose)
            self.shutdown_flag = True
            self.get_logger().warn('init pose sent')
        else:
            # wait for amcl launch
            return
    
    def run(self):
        while rclpy.ok() and not self.shutdown_flag:
            rclpy.spin_once(self)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = InitPosePublisher()
    node.run()

if __name__ == '__main__':
    main()
