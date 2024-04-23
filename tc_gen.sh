#!/bin/bash 

<<EOF
format of seed:
topic_name \n
topic_type \n
contents
EOF


function on_exit {
  echo "[Fuzz][TCGen] exit"
  exit 0 # cmd at blocking systemcalls may not response
}
trap on_exit SIGINT SIGTERM

redis subscribe "HeartBeat" | while read type; do 
  read key
  read round
  [ "$type" != "message" ] && continue

  seed="$(redis ZRevRange "Pool" 0 0)"
  # if pool is used up, exit 1
  if [ -z "$seed" ]; then
    echo "[Fuzz][TCGen] pool used up" > "$ERROR" 
    echo "[Fuzz][TCGen] exit"
    exit 1
  fi
  score=$(redis ZScore "Pool" "$seed")
  echo "[Fuzz][TCGen] new testcase: $seed, $score"

  # mutation by three times
  quiet redis-cli ZAdd "Pool" $((score-1)) "1_$seed"
  quiet redis-cli ZAdd "Pool" $(($score-1)) "2_$seed"
  quiet redis-cli ZAdd "Pool" $(($score-1)) "3_$seed"

  # remove seed with score -lt 0
  quiet redis-cli ZRemRangeByScore "Pool" -inf 0 > /dev/null

  # when szie > 200, remove seed with least score
  while [ $(redis-cli ZCard "Pool") -gt 200 ]; do
    quiet redis-cli ZRemRangeByRank "Pool" 0 0  
  done
  sleep 1
done
