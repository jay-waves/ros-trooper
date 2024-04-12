#!/bin/bash

TOPIC_NAME="/chatter"

MSG_TYPE=".."

while IFS= read -r MSG; do
  ros2 topic pub $TOPIC_NAME $MSG_TYPE "data: \"MSG_CONTENT\"" --once
done

