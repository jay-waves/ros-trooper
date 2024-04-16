#!/bin/bash 

declare -a childs

function on_error {
  echo "[Fuzz][ROSDisp] exit"
  for pgid in "${childs[@]}"; do
    kill -SIGTERM -- -"$pgid"
    sleep 1
    kill -SIGKILL -- -"$pgid"
  done
  exit 1
}

function on_exit {
  echo "[Fuzz][ROSDsp] exit"
  for pgid in "${childs[@]}"; do
    kill -SIGKILL -- -"$pgid"
  done
  exit 0
}
trap on_exit SIGINT SIGTERM

redis-cli subscribe heartbeat | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$((round % $THRESHOLD))" -ne 1 ] && continue 
  [ "$round" -eq -1 ] && on_exit

  if [ ${#childs[@]} -gt 0 ]; then
    oldest=${childs[0]}
    kill $oldest; wait $oldest
    quiet redis-cli publish target "$oldest"
    echo "[Fuzz][ROSDsp] killing oldest target: $oldest"
    childs=("${childs[@]:1}")
  fi

  # dispatcher new process, !!may datarace at &
  guard "$TARGET" pid
  childs+=($pid) 
  echo "[Fuzz][ROSDsp] dispatch new target: $pid"

  sleep 1
done
