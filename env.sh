export THRESHOLD=30
export ERROR="/tmp/fuzz_error"   # fuzz error catch
export TESTEE="/tmp/fuzz_testee" # contain testee pid
[[ -p $ERROR ]] && rm -f $ERROR
[[ -p $TESTEE ]] && rm -f $TESTEE
mkfifo $ERROR $TESTEE

# for redis
# heartbeat channel
# pool: sorted sets

function quiet { # make command silent
  "$@" > /dev/null 2>&1
  return $?
}

function guard { # guard subroutine in background
  local cmd="$1"
  local -n pid_var=$2
  $cmd &
  pid_var=$!
}

function wait_heartbeat_depublicated {
  #! notice, inotifywait will block trap of signal!!!
  # do not use inotifywait, use socket
  inotifywait -m -e modify $HEARTBEAT 2>&1 > /dev/null &
  inotify_pid=$!
  trap "kill $inotify_pid; exit" SIGINT # this will not work
  wait $inotify_pid
  case $? in
    0)
      read round < $HEARTBEAT
      ;;
    *)
      echo "[TCGEN] inotifywait error" > $ERROR
      exit 1
      ;;
  esac
}

export -f quiet guard 
