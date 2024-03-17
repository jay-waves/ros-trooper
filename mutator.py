from myrozz.msg_parser import MsgNode
from random import randint, choices
from copy import deepcopy, copy

'''bytes mutator'''
def byte_replace(data: bytearray):
  if not data:
    return False
  index = randint(0, len(data) - 1)
  new_byte = randint(0, 255)
  data[index] = new_byte
  return True

def bytes_swap(data: bytearray):
  if len(data) < 2:
    return False
  idx1 = randint(0, len(data) - 1)
  idx2 = randint(0, len(data) - 1)
  if idx1 == idx2:
      return False
  data[idx1], data[idx2] = data[idx2], data[idx1]
  return True

def bit_flip(data: bytearray):
  if not data:
    return False
  byte_index = randint(0, len(data) - 1)
  bit_index = randint(0, 7)
  bit_mask = 1 << bit_index
  data[byte_index] ^= bit_mask
  return data

def bmutator(data:bytearray, wghts=None)->bool:
  # bmutator -> byte mutator
  fns = [bit_flip, bytes_swap, byte_replace, lambda _: True] 
  if wghts is None:
      wghts = [1] * len(fns) 
  elif len(wghts)!=len(fns):
    return False
  for _ in range(10):
    fn = choices(fns, weights=wghts, k=1)[0]
    if fn(data):
      return True
  else:
    return False

####################################
'''
data mutator: 
  transform different type to bytes, decide mutation weights
'''

TYPE_FMT = {
    'uint8': 'B',  # unsigned char
    'int8': 'b',   # signed char
    'int16': 'h',  # short
    'uint16': 'H', # unsigned short
    'int32': 'i',
    'uint32': 'I',
    'int64': 'q',  # long long
    'uint64': 'Q', # unsigned long long
    'float64': 'd',
    'float32': 'f',
    'string': 's',
    'bool': '?',
}

import struct

def bool_dmutator(data: bool):
  # dmutator -> data mutator
  return choices([data, not data], [9, 1], k=1)[0]

def list_dmutator(data: list, fn, size=None):
  pass

def dmutator(data, type):
  #! donot put into str, list or bool, just donot care
  fmt = TYPE_FMT.get(type, None)
  if fmt is None or type in ['string', 'bool']:
    raise ValueError("invalid datatype")
  packedata: bytearray = struct.pack(fmt, data)
  if bmutator(packedata, [1, 1, 1, 9]):
    return struct.unpack(fmt, packedata)[0]

##############################

'''
builtin_interfaces/Time
  int32 sec
  uint32 nanosec
'''
def time_mutator(msg: dict):
  # avoid using deepcopy() causing wastes.
  # only visit direct sons, only list need deepcopy
  new_msg = copy(msg)
  new_msg['sec'] = dmutator(msg['sec'], 'int32')
  new_msg['nanosec'] = dmutator(msg['nanosec'], 'uint32') 
  return new_msg

'''
std_msgs/Header
	builtin_interfaces/Time stamp
	string frame_id
'''
def header_mutator(msg: dict):
  new_msg = copy(msg)
  #TODO too may deepcopy calling other mutators
  new_msg['stamp'] = time_mutator(msg['stamp'])
  # skip frame_id field
  return new_msg

'''
bond/msg/Sattus
  std_msgs/Header header
  string id 
  string instance_id 
  bool active
  float32 heartbeat_timeout
  float32 heartbeat_period
'''
def sattus_mutator(msg: dict):
  new_msg = copy(msg) 
  new_msg['header'] = header_mutator(msg['header'])
  # skip id field
  # skip instance_id field
  new_msg['active'] = bool_dmutator(msg['active'])
  new_msg['heartbeat_timeout'] = dmutator(msg['heartbeat_timeout'], 'float32')
  new_msg['heartbeat_period'] = dmutator(msg['heartbeat_period'], 'float32')
  return new_msg

'''
Vector3
	float64 x
	float64 y
	float64 z
'''
def vector3_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['x'] = dmutator(msg['x'], 'float64')
  new_msg['y'] = dmutator(msg['y'], 'float64')
  new_msg['z'] = dmutator(msg['z'], 'float64')
  return new_msg

'''
geometry_msgs/msg/Twist
  Vector3  linear
  Vector3  angular
'''
def twist_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['linear'] = vector3_mutator(msg['linear'])
  new_msg['angular'] = vector3_mutator(msg['angular'])
  return new_msg

'''
geometry_msgs/TwistWithCovariance twist
	Twist twist
	float64[36] covariance
'''
def twistwithcovariance_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['twist'] = twist_mutator(msg['twist'])
  new_msg['covariance'] = list_dmutator(msg['covariance'], 36,...)
  return new_msg

'''
Point32
	float32 x
	float32 y
	float32 z
'''
def point32_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['x'] = dmutator(msg['x'], 'float32')
  new_msg['y'] = dmutator(msg['y'], 'float32')
  new_msg['z'] = dmutator(msg['z'], 'float32')
  return new_msg

'''
Point64
	float64 x
	float64 y
	float64 z
'''
def point64_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['x'] = dmutator(msg['x'], 'float64')
  new_msg['y'] = dmutator(msg['y'], 'float64')
  new_msg['z'] = dmutator(msg['z'], 'float64')
  return new_msg

'''
geometry_msgs/msg/Polygon
  Point32[] points
'''
def polygon_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['points'] = list_dmutator(msg['points'])
  return new_msg

'''
Quaternion orientation
  float64 x 0
  float64 y 0
  float64 z 0
  float64 w 1
'''
def quaternion_mutator(msg: dict):
  #? 有默认值怎么解决?
  new_msg = copy(msg)
  new_msg['x'] = dmutator(msg['x'], 'float64')
  new_msg['y'] = dmutator(msg['y'], 'float64')
  new_msg['z'] = dmutator(msg['z'], 'float64')
  new_msg['w'] = dmutator(msg['w'], 'float64')
  return new_msg

'''
geometry_msgs/Pose origin
  Point position
  Quaternion orientation
'''
def pose_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['position'] = point64_mutator(msg['position'])
  new_msg['orientation'] = quaternion_mutator(msg['orientation'])
  return new_msg

'''
geometry_msgs/PoseStamped
	std_msgs/Header header
	Pose pose
'''
def posestamped_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['pose'] = pose_mutator(msg['pose'])
  return new_msg

'''
geometry_msgs/PoseWithCovariance pose
	Pose pose
	float64[36] covariance
'''
def posewithcovariance_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['pose'] = pose_mutator(msg['pose'])
  new_msg['covariance'] = list_dmutator(msg['covariance'], 36, ...)
  return new_msg

'''
MapMetaData info
	builtin_interfaces/Time map_load_time
	float32 resolution
	uint32 width
	uint32 height
	geometry_msgs/Pose origin
'''
def mapmetadata_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['map_load_time'] = time_mutator(msg['map_load_time'])
  new_msg['resolution'] = dmutator(msg['resolution'], 'float32')
  new_msg['width'] = dmutator(msg['width'], 'uint32')
  new_msg['height'] = dmutator(msg['height'], 'uint32')
  new_msg['origin'] = point32_mutator(msg['origin'])
  return new_msg

'''
nav_msgs/msg/OccupancyGrid
  std_msgs/Header header
  MapMetaData info
  int8[] data
'''
def occupancygrid_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['info'] = mapmetadata_mutator(msg['info'])
  new_msg['data'] = list_dmutator(msg['data'])
  return new_msg

'''
CostmapMetaData metadata
	builtin_interfaces/Time map_load_time
	builtin_interfaces/Time update_time
	string layer
	float32 resolution
	uint32 size_x
	uint32 size_y
	geometry_msgs/Pose origin
'''
def costmapmetadata_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['map_load_time'] = time_mutator(msg['map_load_time'])
  new_msg['update_time'] = time_mutator(msg['update_time'])
  # skip layer field
  new_msg['resolution'] = dmutator(msg['resolution'], 'float32')
  new_msg['size_x'] = dmutator(msg['size_x'], 'uint32')
  new_msg['size_y'] = dmutator(msg['size_y'], 'uint32')
  new_msg['origin'] = pose_mutator(msg['origin'])
  return new_msg

'''
/local_costmap/costmap_raw, nav2_msgs/msg/Costmap
  std_msgs/Header header
  CostmapMetaData metadata
  uint8[] data
'''
def costmap_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['metadata'] = costmapmetadata_mutator(msg['metadata'])
  new_msg['data'] = list_dmutator(msg['data'])
  return new_msg

'''
map_msgs/msg/OccupancyGridUpdate
  std_msgs/Header header
  int32 x
  int32 y
  uint32 width
  uint32 height
  int8[] data
'''
def occupancygridupdate_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['x'] = dmutator(msg['x'], 'int32')
  new_msg['y'] = dmutator(msg['y'], 'int32')
  new_msg['width'] = dmutator(msg['width'], 'uint32')
  new_msg['height'] = dmutator(msg['height'], 'uint32')
  new_msg['data'] = list_dmutator(msg['data'],...)
  return new_msg

'''
nav_msgs/msg/Path
  std_msgs/Header header
  geometry_msgs/PoseStamped[] poses
'''
def path_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['poses'] = list_dmutator(msg['poses'])
  return new_msg

'''
sensor_msgs/msg/LaserScan
  std_msgs/Header header 
  float32 angle_min            
  float32 angle_max            
  float32 angle_increment      
  float32 time_increment       
  float32 scan_time            
  float32 range_min            
  float32 range_max            
  float32[] ranges             
  float32[] intensities        
'''
def laserscan_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  new_msg['angle_min'] = dmutator(msg['angle_min'], 'float32')
  new_msg['angle_max'] = dmutator(msg['angle_max'], 'float32')
  new_msg['angle_increment'] = dmutator(msg['angle_increment'], 'float32')
  new_msg['time_increment'] = dmutator(msg['time_increment'], 'float32')
  new_msg['scan_time'] = dmutator(msg['scan_time'], 'float32')
  new_msg['range_min'] = dmutator(msg['range_min'], 'float32')
  new_msg['range_max'] = dmutator(msg['range_max'], 'float32')
  new_msg['ranges'] = list_dmutator(msg['ranges'], ...)
  new_msg['intensities'] = list_dmutator(msg['intensities'], ...)
  return new_msg

'''
Transform transform
  Vector3 translation
  Quaternion rotation
'''
def transform_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['translation'] = vector3_mutator(msg['translation'])
  new_msg['rotation'] = quaternion_mutator(msg['rotation'])
  return new_msg

'''
geometry_msgs/TransformStamped
	std_msgs/Header header
	string child_frame_id
	Transform transform
'''
def transformstamped_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  # skip child_frame_id field
  new_msg['transform'] = transform_mutator(msg['transform'])
  return new_msg

'''
nav_msgs/msg/Odometry
  std_msgs/Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
'''
def odometry_mutator(msg: dict):
  new_msg = copy(msg)
  new_msg['header'] = header_mutator(msg['header'])
  # skip child_frame_id field
  new_msg['pose'] = posewithcovariance_mutator(msg['pose'])
  new_msg['twist'] = twistwithcovariance_mutator(msg['twist'])
  return new_msg