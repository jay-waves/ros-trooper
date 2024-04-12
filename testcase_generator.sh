#!/bin/bash

redis-server &
REDIS_PID=$!
if ! redis-cli ping; then
  echo "启动Redis失败"
  exit 1
fi

trap "kill $REDIS_PID" EXIT

load_testcases() {
  local testcase_dir="seeds"
  local testcase_key="testcases"
  for seed_
}


mutator(){
  local testcase=$(cat -)
  echo "$testcase"
}

on_new_fuzz_round() {

}

echo msg | mutate > "hello.txt"


