'''将关闭职责交给 fuzz.py'''

import sys
from typing import Tuple
from os import environ

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable,
    TimerAction, EmitEvent, IncludeLaunchDescription, LogInfo, GroupAction, ExecuteProcess)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

from ...config import FUZZ_HOME

# switch
use_collector = "False"
use_odom_interceptor = "True"
use_scan_interceptor = "True"
use_covr = 'True' # todo

output = 'screen' # log, both, screen
wait_for_pose_public = '5'

# wait for finishing time
wait_for_nav2=10.0
wait_for_pose_public=5.0
wait_for_this_circle_finishing=60.0 
#wait_for_this_circle_finishing=30.0 #脚本调试，暂用30s
wait_for_this_circle_cleanup=15.0


def generate_launch_description():
  namespace = environ.get("FUZZ_NAMESPACE")

  def output_to_stderr(info):
      def wrapper(context, *args, **kwargs):
          print(info, file=sys.stderr) 
      return OpaqueFunction(function=wrapper)

  # workspace dir
  bringup_dir = get_package_share_directory('bringup')
  # luanch description, using for launch ros
  ld = LaunchDescription()

  # Include nav2 launch file
  nav2_bringup_dir = get_package_share_directory('nav2_bringup')
  tb3_simulation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(f'{nav2_bringup_dir}/launch/tb3_simulation_launch.py'),
      launch_arguments={
        'namespace': namespace,
        'headless': 'False' , # 是否关闭 gazebo GUI
        'use_composition': 'False' # 是否在容器中运行节点, ASan 无法兼容
      }.items()
  )
  launch_nav2 = GroupAction([
      output_to_stderr("[Launcher]:Start the Navigation2"),
      tb3_simulation_launch,
  ])


  # public 2D_pose_estimate
  #public_initial_pose = GroupAction([
  #    output_to_stderr("[Launcher]:Set the Initial Pose by 2D_pose_estimate"),
  #    output_to_stderr(f"[Launcher]:Wait Initial Pose for {wait_for_pose_public} seconds"),
  #    Node(
  #        package='bringup',
  #        name='init_pose_publisher',
  #        executable='init_pose_publisher',
  #        output=output, # log, both, screen
  #    ),
  #    TimerAction( 
  #        period=wait_for_pose_public,
  #        actions=[
  #            output_to_stderr("[Launcher]:Set the Initial Pose by 2D_pose_estimate Successfully"),
  #        ]
  #    )
  #])
  
  '''
  # launch msg interceptor
  launch_interceptor = GroupAction([
      #output_to_stderr("[Launcher]:Start the Interceptor Nodes!"),
      # Node: scan_interceptor
      Node(
          condition=IfCondition(use_scan_interceptor),
          package='interceptor',
          name='scan_interceptor',
          executable='scan_interceptor',
          # arguments=['--ros-args', '--log-level', 'info'],
          output=output, 
      ),
      # Node: odom_interceptor
      Node(
          condition=IfCondition(use_odom_interceptor),
          package='interceptor',
          name='odom_interceptor',
          executable='odom_interceptor',
          # arguments=['--ros-args', '--log-level', 'info'],
          output=output, # log, both, screen
      ),
      output_to_stderr("[Launcher]:Start the Interceptor Nodes Scuccessfully!")
  ])
  '''


  # Public Nav2 Goal
  #public_goal_pose = GroupAction([
  #    output_to_stderr("[Launcher]:Publishing goal pose for gazebo..."),
  #    output_to_stderr("[Launcher]:Sart Fuzzing ^_^ ^_^ ^_^ ^_^"),
  #    Node(
  #        package='bringup',
  #        name='goal_pose_publisher',
  #        executable='goal_pose_publisher',
  #        # parameters=[f'{bringup_dir}/params/goals.yaml'],
  #        output=output, # log, both, screen
  #    ),
  #])


  start_fuzz = GroupAction([
      output_to_stderr("[Launcher]:Start Launcher!")
  ])

  # 制定执行顺序
  ld.add_action(start_fuzz)
  ld.add_action(launch_nav2) 

  return ld
