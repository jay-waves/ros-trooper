#!/bin/bash
source /opt/ros/iron/setup.bash
source $HOME/src/nav2_iron/install/setup.bash
#export TURTLEBOT3_MODEL=burger
#export TURTLEBOT3_MODEL=waffle
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# export ROS_LOG_DIR="ros_log"
# export ASAN_OPTIONS=halt_on_error=0:new_delete_type_mismatch=0:detect_leaks=0


cleanup(){
  echo "Cleanup and exit"
  jobs -p | xargs -r kill
  exit 0
}
trap cleanup SIGINT

ros2 launch nav2_launch.py & 
sleep 1

# Publish the initial position
./publish_pose.py 
sleep 1

./publish_goal.py &
wait
