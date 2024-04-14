#!/bin/bash
export NUM_SEED=30
export HEARTBEAT="/tmp/fuzz_heartbeat"
export ERROR="/tmp/fuzz_error"
# seed path: seeds

# variables
if [[ ! -p $ERROR ]]; then
  mkfifo $ERROR
fi
declare -A pids
round=0
touch $HEARTBEAT

function cleanup {
  echo "[Fuzz] exit, please wait..."
	timeout 5 redis-cli shutdown
  kill -SIGINT ${pids[tcg]}
  wait ${pids[tcg]}
	rm $HEARTBEAT
  rm $ERROR
	exit 0
}
trap cleanup SIGINT SIGTERM

( # catch error fifo
  while true; do
    if read line < $ERROR; then
      echo "[Fuzz][Error] $line"
      kill -SIGINT $$
      break
    fi
  done
) &


function guard {
  local cmd="$1"
  local -n pid_var=$2
  $cmd &
  pid_var=$!
}

function load_data {
  redis-server --port 6379 &
  while [ "$(redis-cli PING)" != "PONG" ]; do
    sleep 1
  done
  echo "[Fuzz] redis server is ready"

  for seed_file in seeds/*; do 
    seed=$(<"$seed_file") 
    if [ -z $(redis-cli ZScore pool "$seed") ]; then
      # 如不存在, 新建
      redis-cli ZAdd pool 3 "$seed"
    fi
  done
  echo "[Fuzz] successfully load seeds"
}


load_data

guard "./testcase_generator.sh" pids[tcg] 
sleep 1 # wait for subroutines

while true; do
	((round++))
  echo -e "\n[Fuzz] round: $round"
	echo "$round" > $HEARTBEAT
  sleep 5
done
