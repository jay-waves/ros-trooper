#!/bin/bash 

declare -a childs

function on_error {
  echo "[Fuzz][Disp] exit"
  echo "-1" > $TESTEE # tell monitor to exit
  if [ ${#childs[@]} -gt 0 ]; then
    kill -SIGTERM ${childs[@]}
    wait ${childs[@]}
  fi
  exit 1
}

function on_exit {
  echo "[Fuzz][Dsp] exit"
  echo "-1" > $TESTEE # tell monitor to exit
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
    echo $oldest > $TESTEE # tell monitor
    kill $oldest
    echo "[Fuzz][Dsp] killing oldest target: $oldest"
    wait $oldest
    childs=("${childs[@]:1}")
  fi

  # dispatcher new process, !!may datarace at &
  $TARGET & pid=$!
  childs+=($pid) 
  echo "[Fuzz][Dsp] dispatch new target: $pid"

  # pop one seed from seed pool
  seed="$(redis-cli --raw ZRevRange pool 0 0)"
  if [ -z "$seed" ]; then
    timeout 1 echo "[Fuzz][Dsp] pool used up" > "$ERROR"
    on_error
  fi
  score=$(redis-cli --raw ZScore pool "$seed")
  quiet redis-cli ZRem pool "$seed"
  quiet redis-cli HMSet "testcases:$pid" testcase "$seed" score "$score"

  sleep 1
done
