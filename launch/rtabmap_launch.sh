#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/JayWaves/src/rtabmap/install/setup.bash
source /home/JayWaves/src/nav2_240315/install/setup.bash
export TURTLEBOT3_MODEL=waffle

# launch command definitions
cmd_gazebo="ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
cmd_rtabmap="ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=false \
    frame_id:=base_footprint \
    subscribe_scan:=true depth:=false \
    approx_sync:=true \
    odom_topic:=/odom \
    scan_topic:=/scan \
    qos:=2 \
    args:='-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1' \
    use_sim_time:=true \
    rviz:=false"
cmd_nav2="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True"
cmd_rviz="ros2 launch nav2_bringup rviz_launch.py"
cmd_relay="ros2 run topic_tools relay /rtabmap/map /map"

# Start a process and capture its output and PID
start_proc() {
  local cmd=$1
  local var_out=$2
  local var_pid=$3

  # Create a temp file in memory to capture process output
  local temp=$(mktemp -p /dev/shm)
  eval $var_out="'$temp'"
  
  # Execute the command in the background, redirecting output
  eval "$cmd > '$temp' 2>&1 &"
  eval $var_pid="'$!'"
}

cleanup() {
  echo "Cleaning up..."
  # Kill all started processes
  kill $pid_gazebo $pid_rtabmap $pid_relay $pid_nav2 $pid_rviz $pid_tail 2>/dev/null
  # Wait for all processes to exit
  wait $pid_gazebo $pid_rtabmap $pid_relay $pid_nav2 $pid_rviz $pid_tail 2>/dev/null
  # Remove output files
  rm -f $out_gazebo $out_rtabmap $out_nav2 $out_relay $out_rviz
  echo "Exited."
}

# Trap SIGINT (Ctrl+C), calling cleanup function
trap cleanup SIGINT

# Start processes
start_proc "$cmd_gazebo" out_gazebo pid_gazebo
start_proc "$cmd_rtabmap" out_rtabmap pid_rtabmap
start_proc "$cmd_nav2" out_nav2 pid_nav2
start_proc "$cmd_rviz" out_rviz pid_rviz
start_proc "$cmd_relay" out_relay pid_relay

# Monitor output in foreground
nice -n 19 tail -f $out_gazebo $out_rtabmap $out_nav2 $out_relay $out_rviz
pid_tail=$!

# Wait for tail to finish before exiting
wait $pid_tail

