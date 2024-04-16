#!/bin/bash 

export ATTCK="sleep 60"
export TARGET="./launch/nav2_fuzz.sh"
source "./env.sh"

# variables
declare -A pids
round=0

function on_exit {
  echo "[Fuzz] exit, please wait..."
	rm $ERROR # avoid i/o blocking on error catch fifo
  if kill -0 ${pids[err]} 2>/dev/null; then
    kill -SIGINT ${pids[err]} 
  fi # kill error fifo monitor
  echo "[Fuzz] emit last heartbeat: -1"
  quiet redis-cli publish heartbeat "-1"
	redis-cli shutdown # avoid blocking of redis subscription
  wait ${pids[tc_gen]} ${pids[mnt]} ${pids[disp]}
	exit 0
}
trap on_exit SIGINT SIGTERM

( # catch error fifo
  trap 'exit 0' SIGINT # process signal
  while true; do
    if read line < $ERROR; then # blocking here
      echo "<error> $line"
      kill -SIGINT $$ # kill fuzz.sh, call on_exit()
      break
    fi
  done
) &
pids[err]=$!

function load_data { # open redis-server, load seeds from ./seeds/*
  redis-server --port 6379 &
  while [ "$(redis-cli PING)" != "PONG" ]; do
    sleep 1
  done
  echo "[Fuzz] redis server is ready"

  for seed_file in seeds/*; do 
    seed=$(<"$seed_file") 
    quiet redis-cli ZAdd pool NX 2 "$seed"
  done
  echo "[Fuzz] successfully load seeds"
}


load_data

guard "./dispatcher.sh" pids[disp]
guard "./tc_gen.sh" pids[tc_gen] 
guard "./monitor.sh" pids[mnt]
sleep 1 # wait for subroutines

while true; do
	((round++))
  echo -e "\n[Fuzz] round: $round"
  quiet redis-cli publish heartbeat "$round"
  sleep 5
done
