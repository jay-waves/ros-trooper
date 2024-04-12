#!/bin/bash

source /opt/ros/humble/setup.bash 

cleanup() {
  jobs -p | xargs -r kill
  echo "cleanup and exit.."
  exit 0
}

trap cleanup SIGINT


get_random_pose(){
  # generate random between -3~3
  python3 -c "from random import uniform; \
    print(f'{uniform(-3, 3):.2f}')"
}

while true; do
  x=$(get_random_pose)
  y=$(get_random_pose)
  echo "moving to ($x, $y)"
  # Publish the goal position
  ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose " 
  pose:
    header:
      frame_id: map
    pose:
      orientation:
        x: 0.0
        y: 0.0
        z: -0.37
        w: 1.0
      position:
        x: $x
        y: $y 
        z: 0.0" &
  wait $!
  sleep 2
done
