#!/bin/bash
<<EOF
testee: topic attacker
ros2_program: target
EOF

declare -A pids
alias redis='redis-cli --raw'

function on_err {
  echo "[Fuzz][Mnt] exit"
  exit 1
}

function on_exit {
  echo "[Fuzz][Mnt] exit"
  exit 0
}
trap on_exit SIGINT SIGTERM

( #callback of heartbeat
  redis subscribe heartbeat | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$round" -eq -1 ] && on_exit
  sleep 1
done
) & hrt_cb=$!

( # callback of asan jnotifywait
  trap 'exit 0' SIGINT
  inotifywait -m "$MONITOR_DIR" -e create -e moved_to |
  while read path action file; do
    echo "[Fuzz][Mnt] New asan detected: $file"
    
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

    # save bugs, update seedpool
    sleep 1
  done
) & dsp_cb=$!

( # callback of ros2 target death
  redis subscribe target | while read type; do
  read key; read pid
  [ "$type" != "message" ] && continue
  quiet redis-cli RPop attcks $THRESHOLD  
  sleep 1
done
) & trgt_cb=$!

