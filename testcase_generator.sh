#!/bin/bash 
function mutator {
  read in
	echo "${in}_mutated"
}

function guard {
  "$@" > /dev/null 2>&1
  return $?
}

function cleanup {
  touch $HEARTBEAT # end inotifywait
  exit 0
}
trap cleanup SIGINT SIGTERM

while guard inotifywait -e modify "$HEARTBEAT"; do 
  case $? in
    0) # noraml eixt
      read round < $HEARTBEAT

      seed="$(redis-cli --raw ZRevRange pool 0 0)"
      # if pool is used up, exit 1
      if [ -z "$seed" ]; then
        echo "pool used up" > "$ERROR"
        exit 1
      fi
      score=$(redis-cli --raw ZScore pool "$seed")
      echo "$seed, $score"

      # mutate seed
      # new_seed=$(mutator "$seed")
      guard redis-cli ZAdd pool $(($score-1)) "$seed"

      # remove seed with score 0
      guard redis-cli ZRemRangeByScore pool -inf 0 > /dev/null

      # when szie > 200, remove seed with least score
      while [ $(redis-cli ZCard pool) -gt 200 ]; do
        guard redis-cli ZRemRangeByRank pool 0 0  
      done
      sleep 1
      ;;
    *) # timeout
      echo "inotifywait error" > "$ERROR"
      exit 1
      ;;
  esac
done

