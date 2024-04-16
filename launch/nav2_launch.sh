#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/src/nav2_240315/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
#ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

# export ROS_LOG_DIR="ros_log"
export ASAN_OPTIONS=halt_on_error=0:new_delete_type_mismatch=0:detect_leaks=0



cleanup(){
  echo "Cleanup and exit"
  kill $pid_nav2
  wait $pid_nav2
}
trap cleanup SIGINT


ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=True use_composition:=False

pid_nav2=$!
wait $pid_nav2
