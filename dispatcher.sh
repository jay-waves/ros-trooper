#!/bin/bash 

function on_exit {
  echo "[Fuzz][Dsp] exit"
  local childs=$(redis LRange Attcks 0 -1)
  for pgid in "$childs"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][Dsp] interrupt target: $pgid"
      kill -SIGINT -- -"$pgid"
    fi
  done
  for pgid in "$childs"; do
    if kill -0 $pgid 2>/dev/null; then
      echo "[Fuzz][Dsp] may dangling, just kill: $pgid"
      kill -SIGKILL -- -"$pgid"
    fi
  done
  # quiet redis-cli Del Attks
  exit 0
}
trap on_exit SIGINT SIGTERM

redis subscribe "HeartBeat" | while read type; do
  read key
  read round
  [ "$type" != "message" ] && continue

  oldest=$(redis RPop "Attcks")
  if [ -n "$oldest" ]; then
    kill -SIGINT -- -"$oldest"
    wait "$oldest"
    echo "[Fuzz][Dsp] oldest attacker killed: $oldest"
  fi

  # dispatcher new process, !!may datarace, not care
  jguard "$ATTCK"  pid
  quiet redis-cli LPush "Attcks" "$pid" # write only, do not delete
  echo "[Fuzz][Dsp] dispatch new attacker: $pid"

  # pop one seed from seed pool
  seed="$(redis ZRevRange "Pool" 0 0)"
  if [ -z "$seed" ]; then
    echo "[Fuzz][Dsp] pool used up" > "$ERROR"
    on_exit
  fi
  score=$(redis ZScore "Pool" "$seed")
  quiet redis-cli ZRem "Pool" "$seed" # move seed from pool to queue
  quiet redis-cli HMSet "Score" "$pid" "$score"
  quiet redis-cli HMSet "Testcase" "$pid" "$seed"

  sleep 1
done
