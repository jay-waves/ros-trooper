#!/bin/bash 

function on_exit {
  echo "[Fuzz][ROSDsp] exit"
  local childs=$(redis LRange "Trgts" 0 -1)
  for pgid in "$childs"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][ROSDsp] interrupt target: $pgid"
      kill -SIGINT -- -"$pgid"
    fi
  done
  for pgid in "$childs"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][ROSDsp] may dangling, just kill: $pgid"
      kill -SIGKILL -- -"$pgid"
    fi
  done
  quiet redis-cli Del "Trgts"
  exit 0
}

function on_err {
  echo "[Fuzz][ROSDsp] exit"
  for pgid in "${childs[@]}"; do
    kill -SIGKILL -- -"$pgid"
  done
  exit 1
}
trap on_exit SIGINT SIGTERM

redis subscribe "HeartBeat" | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue
  [ "$((round%THRESHOLD))" -ne 1 ] && continue 

  oldest=$(redis RPop "Trgts")
  if [ -n "$oldest" ]; then
    kill -SIGINT -- -"$oldest"
    wait "$oldest"
    echo "[Fuzz][ROSDsp] oldest target killed: $oldest"
    quiet redis-cli publish "TrgtKilled" "$oldest"
  fi

  # dispatcher new process, !!may datarace at &
  jguard "$TARGET" pid
  quiet redis-cli LPush "Trgts" "$pid"
  echo "[Fuzz][ROSDsp] dispatch new target: $pid"

  sleep 1
done
