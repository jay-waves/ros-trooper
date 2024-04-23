#!/bin/bash 
declare -A pids

function on_exit {
  echo "[Fuzz][Mnt] exit"
  for pgid in "${pids[@]}"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][Mnt] interrupt daemon: $pgid"
      kill -SIGINT -- -"$pgid"
    fi
  done
  for pgid in "${pids[@]}"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][Mnt] dangling, just kill: $pgid"
      kill -SIGKILL -- -"$pgid"
    fi
  done
  exit 0
}
trap on_exit SIGINT SIGTERM

set -m
( # callback of heartbeat
  redis subscribe "HeartBeat" | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  sleep 1
done
) & pids[hrt]=$!

( # callback of asan jnotifywait
  inotifywait -m "$ASAN_LOGDIR" -e create -e moved_to |
  while read path action file; do
    echo "[Fuzz][Mnt] New asan detected: $file"
    
<<EOF
    # get latest attck and testcase info
    attck=$(redis LIndex attcks -1)
    testcase=$(redis HGet "testcases:$attck" testcase)
    score=$(redis HGet "testcases:$attck" score)
    if [ -n "$testcase" ]; then
      echo "[Fuzz][Mnt] interesting testcase: $testcase"
      quiet redis-cli Del "testcases:$attck"
    else
      timeout 1 echo "[Fuzz][Mnt] testcase not found" > "$ERROR"
      on_error
    fi
EOF
    # save bugs, update seedpool
    sleep 1
  done
) & pids[attk]=$!

( # callback of ros2 target death
  redis subscribe "TrgtKilled" | while read type; do
  read key; read pid
  [ "$type" != "message" ] && continue
  quiet redis-cli RPop Attcks $THRESHOLD  
  echo "[Fuzz][Mnt] connect attackers to target"
  sleep 1
done
) & pids[trgt]=$!
set +m

wait
