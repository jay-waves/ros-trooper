#!/bin/bash

./nav2_launch.sh > /dev/null &
sleep 2
./publish_pose_goal.sh &
