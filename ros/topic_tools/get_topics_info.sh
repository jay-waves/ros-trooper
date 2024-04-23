#!/bin/bash

source /opt/ros/humble/setup.bash
ros2 topic list | while read topic; do
  echo -en "\n$topic\n"
  ros2 topic info "$topic"
done
