export THRESHOLD=5
export ASAN_LOGDIR="log/sanitizer"
export ASAN_OPTIONS="\
  halt_on_error=0:\
  new_delete_type_mismatch=0:\
  detect_leaks=0:\
  log_path=$ASAN_LOG/asan"

export ERROR="/tmp/fuzz_error"   # fuzz error catch
[[ -p $ERROR ]] && rm -f $ERROR
mkfifo $ERROR 


function quiet { # make command silent
  "$@" > /dev/null 2>&1
  return $?
}

function guard { # guard subroutine in background
  local cmd="$1"
  local -n pid_var=$2 # indirect named call of $2
  eval "$cmd" & pid_var=$!
}

function jguard {
  # guard with job control
  set -m
  local cmd="$1"
  local -n pid_var=$2 # indirect named call of $2
  eval "$cmd" & pid_var=$!
  set +m
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

function redis {
  redis-cli --raw "$@"
}

function log_err {
  echo -e "\033[31m$@\033[0m"
}

export -f quiet guard jguard redis log_error
