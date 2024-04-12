import time
from datetime import datetime

import os
import psutil
import signal
import subprocess
import threading
from contextlib import contextmanager, ExitStack

from .config import log, round, FUZZ_HOME, NAV2_HOME, ROS_HOME

'''src/bringup/launch/launch.py'''				
from myrozz.checker import Checker


ASAN_SWITCH = ("halt_on_error=1:"
               "new_delete_type_mismatch=0:"
               "detect_leaks=0:")
LAUNCH_CMD = (f"source {ROS_HOME}/setup.bash && "
                f"source {NAV2_HOME}/install/setup.bash && "
                f"source {FUZZ_HOME}/install/setup.bash && "
                "ros2 launch bringup fuzz_launch.py")
ROUND_LIMIT = 100000


def fuzz():
  # global setting
  global round
  checker = Checker()
  now = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

  # set global env
  env = os.environ.copy()
  env["TURTLEBOT3_MODEL"] = "waffle"
  env["GAZEBO_MODEL_PATH"] = f"{env.get("GAZEBO_MODEL_PATH", "").strip()}\
                        :/opt/ros/humble/share/turtlebot3_gazebo/models"

  '''Fuzz Circle'''
  while round < ROUND_LIMIT:
    log.info("\033[34m[fuzz]: round %d starts, preparing...\033[0m", round)
    
    # setup log dir for this round
    log_dir = f'{FUZZ_HOME}/log/{now}/round'
    if not os.path.exists(log_dir):
      os.makedirs(log_dir)
    env["ASAN_OPTIONS"] = ASAN_SWITCH + f'log_path={log_dir}/asan'     # set asan options, and log dir
    env['ROS_LOG_DIR'] = log_dir                                       # set ros2 default log dir
    env['FUZZ_NAMESPACE'] = random_namespace()                         # set a random ros2 namespace
    
    log.info("[Fuzz]: ros2 launching...")
    with round_timeout(20), ros2_launch(LAUNCH_CMD, env=env):
      pass
    checker.work()
    round += 1
          
  log.info("[fuzz]: round %d ends, please waiting...", round)
  checker.work()
  # cleanup residual log files
  log.info("[fuzz]: Happing fuzzing, see you. :)")


if __name__ == '__main__':
    fuzz()


#! 似乎这个方法不太行啊
@contextmanager
def round_timeout(timeout=20):
  timer = None
  def timeout_handler():
    time.sleep(timeout)
    raise TimeoutError
  try:
    timer = threading.Thread(target=timeout_handler)
    timer.daemon = True
    timer.start()
    yield
  except TimeoutError:
    log.info("[Fuzz]: round end")
  finally:
    if timer is not None:
      timer.join() # wait for round timeout

#TODO 尝试用 namespace 运行 navigation2
def random_namespace():
  from random import choice
  from string import ascii_lowercase, digits
  whitelist = ascii_lowercase + digits + '_'
  namespace = ''.join(choice(whitelist) for _ in range(10))  
  return namespace

#TODO 测试 ros 程序上下文管理, 转移到 ros2_process_manager
@contextmanager
def ros2_launch(launch_cmd: str, env):
  process = subprocess.Popen(launch_cmd, env=env, start_new_session=True)
  try:
    yield process
  except KeyboardInterrupt:
    log.info("[fuzz]: User exits with ^C")
    #TODO
    normal_ros_exit()
  finally:
    normal_ros_exit()

def normal_ros_exit(process: subprocess.Popen):
  pgid = process.pid
  process.send_signal(signal.SIGINT) # use SIGINT instead of SIGTERM
  try:
    process.wait(3)
  except subprocess.TimeoutExpired:
    log.warn("[fuzz]: Launch process %d didnot terminate in time. \
             Forcing termination.", pgid)
    process.kill() # forced kill

  # check residual processes of process group
  parent = psutil.Process(pgid)
  childs = parent.children(recursive=True)
  for child in childs:
    log.debug("[fuzz]: Cleaning up residual process %d", child.pid)
    child.terminate
    try:
      child.wait(1)
    except psutil.TimeoutExpired:
      log.debug("[fuzz]: Residual process %d didnot terminate in time. \
                Forcing termination.", child.pid)
      child.kill()
  log.info("[fuzz]: ROS2 process exited")


