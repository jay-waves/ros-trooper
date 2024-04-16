#!/bin/bash
<<EOF
1. 监控被测进程, 收集信息
2. 种子权重修改 (避免和 tc_gen 竞争)
3. 
EOF


function on_exit {
  echo "[Fuzz][Mnt] exit"
  exit 0
}
trap on_exit SIGINT SIGTERM

redis-cli subscribe heartbeat | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$round" -eq -1 ] && on_exit

  # May blocking at read, wait dispatcher. It's dispatcher's responsibility
  # to make sure it's reeady for monitor to collect information.
  # To avoid deadlock, dispatcher will send "-1" to monitor when exiting
  read pid < $TESTEE 
  [ $pid -eq -1 ] && on_exit # exit
  echo "[Fuzz][Mnt] watch on testee: $pid"

  testcase=$(redis-cli --raw HGet "testcases:$pid" testcase)
  score=$(redis-cli --raw HGet "testcases:$pid" score)
  if [ -n "$testcase" ]; then
    echo "[Fuzz][Mnt] get testcase: $testcase"
    quiet redis-cli Del "testcases:$pid"
  else
    timeout 1 echo "[Fuzz][Mnt] testcase not found" > "$ERROR"
    echo "[Fuzz][Mnt] exit"
    exit 1
  fi

  # collect information, if anything interesting, 


  sleep 1
done
