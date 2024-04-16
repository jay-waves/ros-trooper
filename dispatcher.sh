#!/bin/bash 

declare -a childs

function on_error {
  echo "[Fuzz][Disp] exit"
  if [ ${#childs[@]} -gt 0 ]; then
    kill -SIGTERM ${childs[@]}
    wait ${childs[@]}
  fi
  exit 1
}

function on_exit {
  echo "[Fuzz][Dsp] exit"
  if [ ${#childs[@]} -gt 0 ]; then
    kill -SIGTERM ${childs[@]}
    wait ${childs[@]}
  fi
  exit 0
}
trap on_exit SIGINT SIGTERM

redis-cli subscribe heartbeat | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$round" -eq -1 ] && on_exit

  if [ ${#childs[@]} -gt 0 ]; then
    oldest=${childs[0]}
    kill $oldest
    wait $oldest
    echo "[Fuzz][Dsp] oldest attacker killed: $oldest"
    quiet redis-cli publish attacker "$oldest"
    childs=("${childs[@]:1}")
  fi

  # dispatcher new process, !!may datarace, not care
  $ATTCK & pid=$!
  childs+=($pid) 
  quiet redis-cli LPush attcks "$pid"
  echo "[Fuzz][Dsp] dispatch new attacker: $pid"

  # pop one seed from seed pool
  seed="$(redis-cli --raw ZRevRange pool 0 0)"
  if [ -z "$seed" ]; then
    timeout 1 echo "[Fuzz][Dsp] pool used up" > "$ERROR"
    on_error
  fi
  score=$(redis-cli --raw ZScore pool "$seed")
  quiet redis-cli ZRem pool "$seed" # remove seed from pool to queue
  quiet redis-cli HMSet "testcases:$pid" testcase "$seed" score "$score"

  sleep 1
done
