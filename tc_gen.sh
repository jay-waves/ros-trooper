#!/bin/bash 
function mutator {
  read in
	echo "${in}_mutated"
}

function on_exit {
  echo "[Fuzz][TCGen] exit"
  exit 0 # cmd at blocking systemcalls may not response
}
trap on_exit SIGINT SIGTERM

redis-cli subscribe heartbeat | while read type; do 
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$round" -eq -1 ] && on_exit

  seed="$(redis-cli --raw ZRevRange pool 0 0)"
  # if pool is used up, exit 1
  if [ -z "$seed" ]; then
    timeout 1 echo "[Fuzz][TCGen] pool used up" > "$ERROR"
    echo "[Fuzz][TCGen] exit"
    exit 1
  fi
  score=$(redis-cli --raw ZScore pool "$seed")
  echo "[Fuzz][TCGen] new testcase: $seed, $score"

  # mutation by three times
  # new_seed=$(mutator "$seed")
  quiet redis-cli ZAdd pool $(($score-1)) "1_$seed"
  quiet redis-cli ZAdd pool $(($score-1)) "2_$seed"
  quiet redis-cli ZAdd pool $(($score-1)) "3_$seed"

  # remove seed with score -lt 0
  quiet redis-cli ZRemRangeByScore pool -inf 0 > /dev/null

  # when szie > 200, remove seed with least score
  while [ $(redis-cli ZCard pool) -gt 200 ]; do
    quiet redis-cli ZRemRangeByRank pool 0 0  
  done
  sleep 1
done
