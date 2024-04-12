#!/bin/bash

DISPATCHER="dispatcher.sh"
GENERATOR="testcase_generator.sh"
MONITOR="monitor.sh"

bash "$MONITOR" &
MONITOR_PID=$!
bash "$DISPATCHER" &
DISPATCHER_PID=$!
bash "$GENERATOR" & && GENERATOR_PID=$!

function cleanup{
  kill $MONITOR_PID
  kill $DISPATCHER_PID
  kill $GENERATOR_PID
  wait $MONITOR_PID
  wait $DISPATCHER_PID
  wait $GENERATOR_PID
  echo "fuzz exited"
  return 0
}
trap cleanup SIGINT

while ture; do
  echo "fuzz round start, generating testcase"
  touch "start_file" #?
  echo "fuzzing in progress..."
  echo "fuzz round ended."
done

