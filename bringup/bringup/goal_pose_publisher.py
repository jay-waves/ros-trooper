import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

'''后续改成可随机发布目标位置'''

class GoalPosePublisher(Node):
    # msg:
    msg_cnt = 6
    # header stamp
    sec = [11, 21, 31, 41, 51, 61, 71, 81, 91]
    nanosec = 0
    # header fram_id
    frame_id = 'map'
    # pose position
    pos_x = [0.5562072396278381, 1.8318827152252197, 0.5013858079910278,
              -0.5272648334503174, 0.6539765000343323, -1.8389906883239746,
              -0.5272648334503174, 0.6539765000343323, -1.8389906883239746]
    pos_y = [-1.7601134777069092, -0.5250500440597534, 0.5395984053611755,
              0.5537899732589722, 1.7773756980895996, 1.5119407176971436,
              0.5537899732589722, 1.7773756980895996, 1.5119407176971436]
    pos_z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # pose orientation
    ori_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ori_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ori_z = [-0.7610554115510837, -0.8918874151604957, -0.9999880007458687,
              -0.4706141831619691, -0.9817747101199641, 0.906607486812718,
              -0.4706141831619691, -0.9817747101199641, 0.906607486812718]
    ori_w = [0.648686874037706, 0.452257491566839, -0.9999880007458687,
              0.882339101823552, 0.19004846373717596, 0.4219749576161213,
              0.882339101823552, 0.19004846373717596, 0.4219749576161213]

    def __init__(self):
        super().__init__('goal_pose_publisher')
        
        # load pose from configuration file
        # self.declare_parameter('poses')
        # data = self.get_parameter('poses').get_parameter_value().string_value
        # self.goals: List = json.loads(data).values()

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.i = 0 # sent msg count
        self.create_timer(10.0, self.publish_pose)
        self.shutdown_flag = False

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.stamp.sec = self.sec[self.i]
        pose.header.stamp.nanosec = self.nanosec
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.pos_x[self.i]
        pose.pose.position.y = self.pos_y[self.i]
        pose.pose.position.z = self.pos_z[self.i]
        pose.pose.orientation.x = self.ori_x[self.i]
        pose.pose.orientation.y = self.ori_y[self.i]
        pose.pose.orientation.z = self.ori_z[self.i]
        pose.pose.orientation.w = self.ori_w[self.i]
        # for _ in range(10):
        self.publisher.publish(pose)
        if self.i == 8:
            self.shutdown_flag = True
        else:
            self.i += 1
    
    def run(self):
        while rclpy.ok() and not self.shutdown_flag:
            rclpy.spin_once(self)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = GoalPosePublisher()
    node.run()

if __name__ == '__main__':
    main()
