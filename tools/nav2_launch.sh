#!/bin/bash
export ROS_LOG_DIR="ros_log"
export ASAN_OPTIONS=halt_on_error=0:new_delete_type_mismatch=0:detect_leaks=0:log_path=asan
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
#ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True use_composition:=False