#!/usr/bin/python3
import subprocess
from multiprocessing import Process, Queue
import time
import signal
import os
import logging as log

type_topics = grouped_topics = {
    'bond/msg/Status': ['/bond'],
    'rosgraph_msgs/msg/Clock': ['/clock'],
    'geometry_msgs/msg/Twist': ['/cmd_vel', '/cmd_vel_nav', '/cmd_vel_teleop'],
    'nav_msgs/msg/OccupancyGrid': ['/downsampled_costmap', '/global_costmap/costmap', '/local_costmap/costmap', '/map'],
    'map_msgs/msg/OccupancyGridUpdate': ['/downsampled_costmap_updates', '/global_costmap/costmap_updates', '/local_costmap/costmap_updates', '/map_updates'],
    'nav2_msgs/msg/Costmap': ['/global_costmap/costmap_raw', '/local_costmap/costmap_raw'],
    'geometry_msgs/msg/Polygon': ['/global_costmap/footprint', '/local_costmap/footprint'],
    'geometry_msgs/msg/PolygonStamped': ['/global_costmap/published_footprint', '/local_costmap/published_footprint'],
    'sensor_msgs/msg/PointCloud2': ['/global_costmap/voxel_marked_cloud', '/local_costmap/voxel_marked_cloud', '/mobile_base/sensors/bumper_pointcloud'],
    'geometry_msgs/msg/PoseStamped': ['/goal_pose'],
    'geometry_msgs/msg/PoseWithCovarianceStamped': ['/initialpose'],
    'sensor_msgs/msg/JointState': ['/joint_states'],
    'nav_msgs/msg/Path': ['/local_plan', '/plan'],
    'nav_msgs/msg/Odometry': ['/odom'],
    'rcl_interfaces/msg/ParameterEvent': ['/parameter_events'],
    'nav2_msgs/msg/ParticleCloud': ['/particle_cloud'],
    'std_msgs/msg/Empty': ['/preempt_teleop'],
    'sensor_msgs/msg/LaserScan': ['/scan'],
    'nav2_msgs/msg/SpeedLimit': ['/speed_limit'],
    'tf2_msgs/msg/TFMessage': ['/tf', '/tf_static'],
    'visualization_msgs/msg/MarkerArray': ['/waypoints']
}

'''哎, ros环境太不稳定了, 自己一个一个抓吧!!
先有针对性爆破几个, 为什么我的rviz2老是崩溃?'''

topic_type = [('/bond', 'bond/msg/Status'),
 ('/clock', 'rosgraph_msgs/msg/Clock'),
 ('/cmd_vel', 'geometry_msgs/msg/Twist'),
 ('/cmd_vel_nav', 'geometry_msgs/msg/Twist'),
 ('/downsampled_costmap', 'nav_msgs/msg/OccupancyGrid'),
 ('/downsampled_costmap_updates', 'map_msgs/msg/OccupancyGridUpdate'),
 ('/global_costmap/costmap', 'nav_msgs/msg/OccupancyGrid'),
 ('/global_costmap/costmap_raw', 'nav2_msgs/msg/Costmap'),
 ('/global_costmap/costmap_updates', 'map_msgs/msg/OccupancyGridUpdate'),
 ('/global_costmap/footprint', 'geometry_msgs/msg/Polygon'),
 ('/global_costmap/published_footprint', 'geometry_msgs/msg/PolygonStamped'),
 ('/global_costmap/voxel_marked_cloud', 'sensor_msgs/msg/PointCloud2'),
 ('/goal_pose', 'geometry_msgs/msg/PoseStamped'),
 ('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
 ('/joint_states', 'sensor_msgs/msg/JointState'),
 ('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid'),
 ('/local_costmap/costmap_raw', 'nav2_msgs/msg/Costmap'),
 ('/local_costmap/costmap_updates', 'map_msgs/msg/OccupancyGridUpdate'),
 ('/local_costmap/footprint', 'geometry_msgs/msg/Polygon'),
 ('/local_costmap/published_footprint', 'geometry_msgs/msg/PolygonStamped'),
 ('/local_costmap/voxel_marked_cloud', 'sensor_msgs/msg/PointCloud2'),
 ('/local_plan', 'nav_msgs/msg/Path'),
 ('/map', 'nav_msgs/msg/OccupancyGrid'),
 ('/map_updates', 'map_msgs/msg/OccupancyGridUpdate'),
 ('/mobile_base/sensors/bumper_pointcloud', 'sensor_msgs/msg/PointCloud2'),
 ('/odom', 'nav_msgs/msg/Odometry'),
 ('/parameter_events', 'rcl_interfaces/msg/ParameterEvent'),
 ('/particle_cloud', 'nav2_msgs/msg/ParticleCloud'),
 ('/plan', 'nav_msgs/msg/Path'),
 ('/preempt_teleop', 'std_msgs/msg/Empty'),
 ('/scan', 'sensor_msgs/msg/LaserScan'),
 ('/speed_limit', 'nav2_msgs/msg/SpeedLimit'),
 ('/tf', 'tf2_msgs/msg/TFMessage'),
 ('/tf_static', 'tf2_msgs/msg/TFMessage'),
 ('/waypoints', 'visualization_msgs/msg/MarkerArray')]

ROS_PREFIX = "source /opt/ros/humble/setup.bash && source /home/JayWaves/src/nav2_240315/install/setup.bash && "

def run_ros_cmd(cmd: str, timeout=10):
  try: # 创建进程组来杀掉 shell 的各个子进程: shell->ros2cli->ros2-daemon
    result = subprocess.run(ROS_PREFIX + cmd, 
                            capture_output=True, text=True, 
                            executable="/bin/bash", shell=True,
                            timeout=timeout, preexec_fn=os.setsid)
    if result.returncode == 0:
      return result.stdout.strip()
    else:
      log.warning("run ros commnd '%s' error: %s", cmd, result.stderr)
  except subprocess.TimeoutExpired:
    os.kill(os.getpid(), signal.SIGTERM)
    log.warning("run ros commnd '%s' error: timeout", cmd)
    # raise TimeoutError("ROS command timed out after {} seconds".format(timeout))

# def get_active_topics():
# 	return run_ros_cmd("ros2 topic list").splitlines()

# def get_topic_type(topic):
# 	return run_ros_cmd("ros2 topic type " + topic)

def get_interface(type, q):
  echo = run_ros_cmd("ros2 interface show --no-comments " + type)
  q.put(f"\n{type}, \n{echo}\n")

def get_topic_msg(topic,q):
  # for --spin-time 0.5, see https://github.com/ros2/ros2cli/issues/248
  echo= run_ros_cmd("time ros2 topic echo --no-daemon --once --full-length --spin-time 5 "+topic)
  q.put(f"\n{topic}, \n{echo}\n")

def main():
  batch_num = 5
  q = Queue()
  global topic_type
	
  while True:
    topic_type1 =  topic_type.copy()
    processes = []
    while topic_type1:
      batch = topic_type1[:batch_num]
      for topic, _ in batch:
        p = Process(target=get_topic_msg, args=(topic, q))
        p.start()
        processes.append(p)
      for p in processes:
        p.join()
      processes.clear()
      while not q.empty():
        print(q.get())
      topic_type1 = topic_type1[batch_num:]
      time.sleep(10)
      print("next batch...")
    print("next round...")
	
if __name__ == "__main__":
	main()