#!/bin/bash 
export ATTCK="sleep 60"
export TARGET="./launch/nav2_fuzz.sh"
source "./env.sh"

# variables
declare -A pids
round=0
set -m

function on_exit {
  echo "[Fuzz] exit, please wait..."
  for pgid in "${pids[@]}"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz] interrupt subroutine: $pgid"
      kill -SIGINT -- -"$pgid"
    fi
  done
  sleep 5
  for pgid in "${pids[@]}"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz] dangling, just kill: $pgid"
      kill -SIGKILL -- -"$pgid"
    fi
  done
	redis-cli shutdown 
  rm $ERROR # remove fifo
	exit 0
}
trap on_exit SIGINT SIGTERM

( # catch error fifo
  trap 'exit 0' SIGINT # process signal
  while true; do
    if read line < $ERROR; then # blocking here
      log_err "<error> $line"
      kill -SIGINT $$ # kill fuzz.sh, call on_exit()
      break
    fi
  done
) & pids[err]=$!

function load_data { # open redis-server, load seeds from ./seeds/*
  redis-server --port 6379 &
  while [ "$(redis-cli PING)" != "PONG" ]; do
    sleep 1
  done
  echo "[Fuzz] redis server is ready"

  for seed_file in seeds/*; do 
    seed=$(<"$seed_file") 
    quiet redis-cli ZAdd "Pool" NX 2 "$seed"
  done
  echo "[Fuzz] successfully load seeds"
}


load_data

guard "./dispatcher.sh" pids[dsp]
guard "./ros_dispatcher.sh" pids[rosdsp]
guard "./tc_gen.sh" pids[tc_gen] 
guard "./monitor.sh" pids[mnt]
sleep 1 # wait for subroutines
set +m

while true; do
	((round++))
  echo -e "\n[Fuzz] round: $round"
  quiet redis-cli publish "HeartBeat" "$round"
  sleep 5
done
