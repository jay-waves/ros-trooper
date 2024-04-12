#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/JayWaves/src/rtabmap/install/setup.bash

topics=$(ros2 topic list)
cmd_echo="ros2 topic echo --no-daemon --once --full-length" # + topic_name
log_path='./log'

function cleanup {
  echo "Exit, cleanup useless log"
  find "$log_path" -type f -size 0
  find "$log_path" -type f -exec grep -q '^WARNING' {} \;
  exited=1
}
trap cleanup SIGINT

for topic in $topics; do
  if [[ $exited ]]; then
    break
  fi
  echo "处理: $topic"
  eval timeout --signal=SIGKILL 3 $cmd_echo $topic >> "$log_path/${topic//\//_}.msg" &
  sleep 1
done

wait $!
cleanup
