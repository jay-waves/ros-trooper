import os
import sys
import subprocess
import psutil
from datetime import datetime
import threading
import logging as log
import time
import signal
from contextlib import contextmanager

from .config import round, FUZZ_HOME, NAV2_HOME, ROS_HOME

# from plugins.check_clean import check_ros2_node_list 		#step 0. pre: 等待ros2节点全部关闭
# from plugins.check_clean import check_ros2_topic_list		#step 0. pre: 等待ros2话题全部关闭
'''src/bringup/launch/launch.py'''				#step 1. Launch: ( nav2 , scan , odom )
from plugins.check_nav2 import wait_nav2_launch			#step 1. 等待nav2启动
from plugins.coverage import Coverage  		#step 4. 覆盖率

# switch
asan_switch = ("halt_on_error=1:"
               "new_delete_type_mismatch=0:"
               "detect_leaks=0:")


def fuzz():
  #-------------------------------------------------- Fuzz Preparation ----------------------------------------------------#
  # time now
  now = datetime.now()
  test_time = now.strftime('%Y-%m-%d_%H-%M-%S')

  # create log dir for this fuzz
  log_dir = f'{FUZZ_HOME}/log/{test_time}'
  if not os.path.exists(log_dir):
    os.makedirs(log_dir)

  # set env
  env = os.environ.copy()
  env["TURTLEBOT3_MODEL"] = "waffle"
  env["GAZEBO_MODEL_PATH"] = env.get("GAZEBO_MODEL_PATH", "") + ":/opt/ros/humble/share/turtlebot3_gazebo/models"
  env["ASAN_OPTIONS"] = asan_switch + f'log_path={log_dir}/r1_asan'	#目录更新我想放在循环头部

  # nav2指令
  source_cmd = (f"source {ROS_HOME}/setup.bash && "
                  f"source {NAV2_HOME}/install/setup.bash && "
                  f"source {FUZZ_HOME}/install/setup.bash && ")
  command = source_cmd + "ros2 launch bringup launch.py"

  #-------------------------------------------------- Fuzz Round ----------------------------------------------------#
  covr = Coverage(log_dir)
  global round
  try:
    while True:
      log.info("[Fuzz]: round %d starts =======================", round)
      log.info("[Fuzz]: preparing...")
      
      #step 0. preparation
      env["ASAN_OPTIONS"] = asan_switch + f'log_path={log_dir}/asan' 
      env["RCUTILS_LOGGING_LEVEL"] = "DEBUG" #????
      
      #step 1. launch(nav2,scan,odom,goal_pose_publisher)  
      log.info("[Fuzz]: start ros2 launch")
      process = subprocess.Popen(command, env=env, 
                              shell=True, executable='/bin/bash', 
                              stdout=subprocess.DEVNULL,  # 丢弃 stdout
                              stderr=sys.stdout, 
                              preexec_fn=os.setsid) # 创建新进程组会话
      pgid = os.getpgid(process.pid) 
      #等待nav2启动
      wati_nav2_period = wait_nav2_launch() #wait_nav2_peirod是等待nav2启动的时间
      
      
      #step 2. Usr_Input: 起始点和目的地  

      #等待 本轮测试 运行 #等待launch进程结束
      exitcode = process.wait()
      if exitcode != 0:
        log.warn("[Fuzz]: fatal error in fuzz round %d", round)
        # raise ValueError('Wrong Subprocess ExitCode') 应允许被测程序出错

      #step 4. update code coverage information
      covr.update() # 更新 code coverage 信息
      round += 1
          
  #-------------------------------------------------- Fuzz End -----------------------------------------------------------#
  except KeyboardInterrupt:
    print(f"[Fuzz]:User Exits with ^C")

    # terminate the subprocesses
    try:
      os.killpg(pgid, signal.SIGTERM)  
      time.sleep(10) # wait for ros cleanup
      try:
        psutil.Process(process.pid)
        os.killpg(pgid, signal.SIGKILL) 
      except psutil.NoSuchProcess:
        pass
    except ProcessLookupError:
      print(f"[Fuzz]:Error Finding the fuzz process. Manually check ros ps")

  except Exception as e:
    print(f"[Fuzz]:Error occurred: {e}")
    print(f"[Fuzz]:Killing all subprocesses...")
    # terminate the subprocess
    os.killpg(pgid, signal.SIGKILL)
      
  finally:
    log.info("[Fuzz]: round %d ends =============================", round)
    log.info("[Fuzz]: waiting...")
    covr.update()
    # check log?
    log.info("[Fuzz]: Happing fuzzing, see you. :)")


if __name__ == '__main__':
    fuzz()


@contextmanager
def fuzz_round(timeout=20):
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

def random_namespace():
  from random import choice
  from string import ascii_lowercase, digits
  whitelist =ascii_lowercase + digits + '_'
  namespace = ''.join(choice(whitelist) for _ in range(10))  
  return namespace

@contextmanager
def ros2_launch_context(launch_file: list, env):
  # 启动launch文件列表
  launch_command = ["ros2", "launch", 'fuzz', launch_file]
  process = subprocess.Popen(launch_command, env=env, start_new_session=True)
  pid = process.pid
  try:
    yield process
  finally:
    # 尝试正常终止launch进程
    process.send_signal(signal.SIGINT)
    try:
      process.wait(10)
    except subprocess.TimeoutExpired:
      print(f"Launch process {pid} did not terminate in time. Forcing termination.")
      # 强制终止
      process.kill()
    # 检查残留进程
    check_residual_processes(pid)


def check_residual_processes(parent_pid):
  parent = psutil.Process(parent_pid)
  children = parent.children(recursive=True)
  for child in children:
    print(f"Cleaning up residual process {child.pid}")
    child.terminate()
    try:
      child.wait(5)
    except psutil.TimeoutExpired:
      print(f"Residual process {child.pid} did not terminate in time. Forcing termination.")
      child.kill()


