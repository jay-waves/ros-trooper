'''
FUZZ_HOME
-> /log
----> /test_time
------> /r1
------> /r2
-> /bringup
'''
import logging

ROS_HOME = '/opt/ros/humble'
NAV2_HOME='/home/JayWaves/src/nav2_240315'
FUZZ_HOME='/home/JayWaves/code/myrozz'
round = 0

log = logging.getLogger("fuzz")
log.setLevel(log.DEBUG)

