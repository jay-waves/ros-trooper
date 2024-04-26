#!/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 设置参数文件和行为树文件的路径
    params_file = LaunchConfiguration('params', default='params.yaml')
    topose_bt_file = LaunchConfiguration('topose_bt', default='topose.xml')
    throughposes_bt_file = LaunchConfiguration('throughposes_bt', default='throughposes.xml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': topose_bt_file,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': throughposes_bt_file
        },
        convert_types=True
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{nav2_bringup_dir}/launch/tb3_simulation_launch.py'),
        launch_arguments={
            'params_file': configured_params,
            'headless': 'True',
            'use_composition': 'False',
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models'),
        nav2_launch,
    ])
