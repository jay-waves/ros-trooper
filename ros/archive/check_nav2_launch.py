#TODO 转移到 ros2_process_manager

import subprocess
import time

WAIT_TIME_LIMIT=20
#会有几个不稳定的：/transform_listener_***
#其他节点有：
node_list='''/transform_listener_
/amcl
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/controller_server
/gazebo
/global_costmap/global_costmap
/intel_realsense_r200_depth_driver
/lifecycle_manager_localization
/lifecycle_manager_navigation
/local_costmap/local_costmap
/map_server
/planner_server
/robot_state_publisher
/rviz
/rviz_navigation_dialog_action_client
/smoother_server
/turtlebot3_diff_drive
/turtlebot3_imu
/turtlebot3_joint_state
/turtlebot3_laserscan
/velocity_smoother
/waypoint_follower
/scan_interceptor
/odom_interceptor
'''
node_list = node_list.split('\n')
dead_nodes = []
def check_node(txt):
     for node_name in node_list:
         if node_name not in txt:
             dead_nodes.append(node_name)
     return True
def wait_nav2_node_launch():
    print("-------node check-------")
    try:
        # 执行命令并捕获输出
        result = subprocess.check_output("ros2 node list", shell=True, stderr=subprocess.STDOUT, text=True)
        check_node(result)
        for dd in dead_nodes:
            print(f"node {dd} not launched")
        return
    except:
        return 
        



#话题：
topic_list='''/amcl/transition_event
/amcl_pose
/behavior_server/transition_event
/bond
/bt_navigator/transition_event
/clicked_point
/clock
/cmd_vel
/cmd_vel_nav
/cmd_vel_teleop
/controller_server/transition_event
/cost_cloud
/diagnostics
/downsampled_costmap
/downsampled_costmap_updates
/evaluation
/global_costmap/costmap
/global_costmap/costmap_raw
/global_costmap/costmap_updates
/global_costmap/footprint
/global_costmap/global_costmap/transition_event
/global_costmap/published_footprint
/global_costmap/voxel_marked_cloud
/goal_pose
/imu
/initialpose
/intel_realsense_r200_depth/camera_info
/intel_realsense_r200_depth/depth/camera_info
/intel_realsense_r200_depth/depth/image_raw
/intel_realsense_r200_depth/image_raw
/intel_realsense_r200_depth/points
/joint_states
/local_costmap/clearing_endpoints
/local_costmap/costmap
/local_costmap/costmap_raw
/local_costmap/costmap_updates
/local_costmap/footprint
/local_costmap/local_costmap/transition_event
/local_costmap/published_footprint
/local_costmap/voxel_grid
/local_costmap/voxel_marked_cloud
/local_plan
/map
/map_server/transition_event
/map_updates
/marker
/mobile_base/sensors/bumper_pointcloud
/odom
/odom_1
/parameter_events
/particle_cloud
/performance_metrics
/plan
/plan_smoothed
/planner_server/transition_event
/preempt_teleop
/received_global_plan
/robot_description
/rosout
/scan
/scan_1
/smoother_server/transition_event
/speed_limit
/tf
/tf_static
/transformed_global_plan
/velocity_smoother/transition_event
/waypoint_follower/transition_event
/waypoints
'''
dead_topic=[]
topic_list = topic_list.split('\n')
def check_topic(txt):
     for topic_name in topic_list:
         if topic_name not in txt:
             dead_topic.append(topic_name)
     return True
def wait_nav2_topic_launch():
    print("-------topic check-------")
    try:
        # 执行命令并捕获输出
        result = subprocess.check_output("ros2 topic list", shell=True, stderr=subprocess.STDOUT, text=True)
        check_topic(result)
        for dd in dead_topic:
            print(f"topic {dd} not launched")
    except:
        return




service_list='''/amcl/change_state
/amcl/get_available_states
/amcl/get_available_transitions
/amcl/get_state
/amcl/get_transition_graph
/behavior_server/change_state
/behavior_server/describe_parameters
/behavior_server/get_available_states
/behavior_server/get_available_transitions
/behavior_server/get_state
/behavior_server/get_transition_graph
/behavior_server/set_parameters_atomically
/bt_navigator/change_state
/bt_navigator/describe_parameters
/bt_navigator/get_available_states
/bt_navigator/get_available_transitions
/bt_navigator/get_state
/bt_navigator/get_transition_graph
/controller_server/change_state
/controller_server/get_available_states
/controller_server/get_available_transitions
/controller_server/get_state
/controller_server/get_transition_graph
/delete_entity
/get_model_list
/global_costmap/clear_around_global_costmap
/global_costmap/clear_entirely_global_costmap
/global_costmap/clear_except_global_costmap
/global_costmap/get_costmap
/global_costmap/global_costmap/change_state
/global_costmap/global_costmap/describe_parameters
/global_costmap/global_costmap/get_available_states
/global_costmap/global_costmap/get_available_transitions
/global_costmap/global_costmap/get_parameter_types
/global_costmap/global_costmap/get_parameters
/global_costmap/global_costmap/get_state
/global_costmap/global_costmap/get_transition_graph
/global_costmap/global_costmap/list_parameters
/global_costmap/global_costmap/set_parameters
/global_costmap/global_costmap/set_parameters_atomically
/lifecycle_manager_localization/describe_parameters
/lifecycle_manager_localization/get_parameter_types
/lifecycle_manager_localization/get_parameters
/lifecycle_manager_localization/is_active
/lifecycle_manager_localization/list_parameters
/lifecycle_manager_localization/manage_nodes
/lifecycle_manager_localization/set_parameters
/lifecycle_manager_localization/set_parameters_atomically
/lifecycle_manager_navigation/describe_parameters
/lifecycle_manager_navigation/get_parameter_types
/lifecycle_manager_navigation/get_parameters
/lifecycle_manager_navigation/is_active
/lifecycle_manager_navigation/list_parameters
/lifecycle_manager_navigation/manage_nodes
/lifecycle_manager_navigation/set_parameters
/lifecycle_manager_navigation/set_parameters_atomically
/local_costmap/clear_around_local_costmap
/local_costmap/clear_entirely_local_costmap
/local_costmap/clear_except_local_costmap
/local_costmap/get_costmap
/local_costmap/local_costmap/change_state
/local_costmap/local_costmap/describe_parameters
/local_costmap/local_costmap/get_available_states
/local_costmap/local_costmap/get_available_transitions
/local_costmap/local_costmap/get_parameter_types
/local_costmap/local_costmap/get_parameters
/local_costmap/local_costmap/get_state
/local_costmap/local_costmap/get_transition_graph
/local_costmap/local_costmap/list_parameters
/local_costmap/local_costmap/set_parameters
/local_costmap/local_costmap/set_parameters_atomically
/map_server/change_state
/map_server/describe_parameters
/map_server/get_available_states
/map_server/get_available_transitions
/map_server/get_parameter_types
/map_server/get_parameters
/map_server/get_state
/map_server/get_transition_graph
/map_server/list_parameters
/map_server/load_map
/map_server/map
/map_server/set_parameters
/map_server/set_parameters_atomically
/pause_physics
/planner_server/change_state
/planner_server/describe_parameters
/planner_server/get_available_states
/planner_server/get_available_transitions
/planner_server/get_parameter_types
/planner_server/get_parameters
/planner_server/get_state
/planner_server/get_transition_graph
/planner_server/list_parameters
/planner_server/set_parameters
/planner_server/set_parameters_atomically
/reinitialize_global_localization
/request_nomotion_update
/reset_simulation
/reset_world
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/rviz/describe_parameters
/rviz/get_parameter_types
/rviz/get_parameters
/rviz/list_parameters
/rviz/set_parameters
/rviz/set_parameters_atomically
/set_camera_info
/smoother_server/change_state
/smoother_server/describe_parameters
/smoother_server/get_available_states
/smoother_server/get_available_transitions
/smoother_server/get_parameter_types
/smoother_server/get_parameters
/smoother_server/get_state
/smoother_server/get_transition_graph
/spawn_entity
/unpause_physics
/velocity_smoother/change_state
/velocity_smoother/get_available_states
/velocity_smoother/get_available_transitions
/velocity_smoother/get_state
/velocity_smoother/get_transition_graph
/waypoint_follower/change_state
/waypoint_follower/get_available_states
/waypoint_follower/get_available_transitions
/waypoint_follower/get_state
/waypoint_follower/get_transition_graph
'''
dead_service = []
service_list = service_list.split('\n')
def check_service(txt):
     for service_name in service_list:
         if service_name not in txt:
             dead_service.append(service.name)
     return True
def wait_nav2_service_launch():
    print("-------service check-------")
    try:
        # 执行命令并捕获输出
        result = subprocess.check_output("ros2 service list", shell=True, stderr=subprocess.STDOUT, text=True)
        check_service(result)
        for dd in dead_service:
            print(f"service {dd} not launched")
    except:
        return



action_list='''/assisted_teleop
/backup
/compute_path_through_poses
/compute_path_to_pose
/drive_on_heading
/follow_path
/follow_waypoints
/navigate_through_poses
/navigate_to_pose
/smooth_path
/spin
/wait
'''
dead_action = []
action_list = action_list.split('\n')
def check_action(txt):
     for action_name in action_list:
         if action_name not in txt:
             dead_action.append(action.name)
     return True
def wait_nav2_action_launch():
    print("-------action check-------")
    try:
        # 执行命令并捕获输出
        result = subprocess.check_output("ros2 action list", shell=True, stderr=subprocess.STDOUT, text=True)
        check_action(result)
        for dd in dead_action:
            print(f"action {dd} not launched")
    except:
        return


def wait_nav2_launch():
    print("=================check nav2 launch state===============")
    wait_nav2_node_launch()
    wait_nav2_topic_launch()
    wait_nav2_service_launch()
    wait_nav2_action_launch()
    print("==================check end ========================")
    
    
def main():
    wait_nav2_launch()
    return 
    
if __name__ == "__main__":
    main()



