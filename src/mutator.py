from random import randint, choices, randrange, shuffle
from copy import deepcopy, copy
import logging as log

def byte_mutator(data:bytearray, knobs=None)->bool:
  '''bytes mutator, keep bytearray in same size'''
  # bmutator -> byte_mutator
  # do not use mutable [] in kargs, all fmutator() will share one [], and 
  # it may be changed by all fmutator()
  def _byte_replace(data: bytearray):
    log.debug(">>>>[bytes mutator]: using byte_replace")
    if not data:
      return False
    index = randint(0, len(data) - 1)
    new_byte = randint(0, 255)
    data[index] = new_byte
    return True

  def _bytes_swap(data: bytearray):
    log.debug(">>>>[bytes mutator]: using bytes_swap")
    if len(data) < 2:
      return False
    idx1 = randint(0, len(data) - 1)
    idx2 = randint(0, len(data) - 1)
    if idx1 == idx2:
        return False
    data[idx1], data[idx2] = data[idx2], data[idx1]
    return True

  def _bit_flip(data: bytearray):
    log.debug(">>>>[bytes mutator]: using bit_flip")
    if not data:
      return False
    byte_index = randint(0, len(data) - 1)
    bit_index = randint(0, 7)
    bit_mask = 1 << bit_index
    data[byte_index] ^= bit_mask
    return data

  fns = [_bit_flip, _bytes_swap, _byte_replace, lambda _: True] 
  if knobs is None:
      knobs = [1] * len(fns) 
  elif len(knobs)!=len(fns):
    raise ValueError("weights must have length 4")
  for _ in range(10):
    fn = choices(fns, weights=knobs, k=1)[0]
    if fn(data):
      return True
  else:
    return False

####################################
'''
[type] mutator: 
  transform different type to bytes, decide mutation weights(knobs)
  now keep mutate:notmutate = 1:10
'''
import struct
from contextlib import contextmanager

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

# bit_flip, bytes_swap, byte_replace, do_nothing
FIELD_KNOBS = [1, 1, 1, 27]
# not data, data
BOOL_KNOBS = [1, 9]
# extend, reverse, shuffle, swap, do_nothing
LIST_STRUC_KNOBS = [1, 1, 1, 1, 1, 45]
# wrapper for FIELD_WGHT used in lmutator, use 1:15
LIST_FIELD_KNOBS = [1, 1, 1, 42]

@contextmanager
def new_field_knobs(wghts):
  '''change global FIELD_WGHTS temporarily'''
  log.debug(">>[FIELD_KNOBS]: temporarily reset FIELD_WGHTS")
  global FIELD_KNOBS
  copy = FIELD_KNOBS.copy()
  FIELD_KNOBS = wghts
  yield # just exit when ErrorRaising
  FIELD_KNOBS = copy
  log.debug(">>[FIELD_KNOBS]: recover FIELD_WGHTS")
  return

def bmutator(data: bool)->bool:
  '''bmutator -> bool mutator'''
  log.debug(">>[bool mutator]")
  return choices([not data, data], BOOL_KNOBS, k=1)[0]

def lmutator(datas: list, fmutator, fixed_size=False):
  '''
  lmutator -> list mutator  
  - fmutator: may be a wrapper for fmutator() or [msg]_mutator(), 
  like: `lambda x: fmutator(x)`
  '''
  def _extend_elem():
    # cover add_elem()'s functionality
    log.debug(">>[list mutator]: using extend_elem")
    num_to_add = randint(1, len(datas))
    # max_num = max(1, len(datas)//3), not change lots at a time
    elem = datas[randrange(0, len(datas))] # just repeat one elem
    insrt_idx = randrange(0, len(datas))
    for _ in range(num_to_add):
      datas.insert(insrt_idx, elem)
  
  def _reverse_elem():
    log.debug(">>[list mutator]: using reverse_elem")
    if len(datas) <2:
      return
    idx1 = randrange(0, len(datas)-1) 
    idx2 = randrange(idx1+1, len(datas))
    #! donot use datas[:].reverse()
    datas[idx1:idx2+1]=datas[idx1:idx2+1][::-1]
  
  def _shuffle_elem():
    log.debug(">>[list mutator]: using shuffle_elem")
    if len(datas)<2:
      return
    idx1 = randrange(0, len(datas)-1) # randrangd: [), randint: []
    idx2 = randrange(idx1+1, len(datas))
    sublist = datas[idx1:idx2+1] # slice: [)
    shuffle(sublist)
    datas[idx1:idx2+1] = sublist

  def _remove_elem():
    log.debug(">>[list mutator]: using remove_elem")
    num_to_remove = randint(1, len(datas))
    idx = randrange(0, len(datas))
    del_end = min(idx+num_to_remove, len(datas))
    del datas[idx:del_end]

  def _swap_elem():
    log.debug(">>[list mutator]: using swap_elem")
    idx1 = randrange(0, len(datas))
    idx2 = randrange(0, len(datas))
    if idx1 == idx2:
      datas[idx1], datas[idx2] = datas[idx2], datas[idx1]

  if not len(datas): # empty list
    return datas
  # only shallow copy list, deepcopy of MutableObject in list[MutabaleObject]
  # is entrusted to **arg: fmutator**'s disposal
  datas = datas[:]
  # Temporarily change global weights settings. Decrease probabilities of 
  # mutation of list elements during lmutator.
  with new_field_knobs(LIST_FIELD_KNOBS):
    for i, data in enumerate(datas):
      # mutate each field, make all elements new (id & contents)
      datas[i] = fmutator(data)

  # mutate list structure
  if not fixed_size:
    fns = [_extend_elem, _reverse_elem, 
           _shuffle_elem, _remove_elem, 
           _swap_elem, lambda:None]
    fn = choices(fns, LIST_STRUC_KNOBS, k=1)[0]
    fn()
  return datas # useless ret, for consistence

def fmutator(data, type):
  log.debug('>>[field mutator]')
  #! donot put into str, list or bool, just donot care
  fmt = TYPE_FMT.get(type, None)
  if fmt is None or type in ['string', 'bool']:
    raise ValueError("invalid datatype")
  packedata = bytearray(struct.pack(fmt, data))
  if byte_mutator(packedata, FIELD_KNOBS):
    return struct.unpack(fmt, packedata)[0]

##############################

@contextmanager
def mutator_context(name: str):
  # For now, just print some field-free information, like mutation schedule
  # info
  try:
    log.debug(f"[{name}]: entry")
    yield
  finally: # print log even ErrorRaising
    log.debug(f"[{name}]: exit")

'''
builtin_interfaces/Time
  int32 sec
  uint32 nanosec
'''
def time_mutator(msg: dict):
  # avoid using deepcopy() causing wastes.
  # only visit direct sons, only list need deepcopy
  with mutator_context("time mutator"):
    new_msg = copy(msg)
    new_msg['sec'] = fmutator(msg['sec'], 'int32')
    new_msg['nanosec'] = fmutator(msg['nanosec'], 'uint32') 
  return new_msg

'''
std_msgs/Header
	builtin_interfaces/Time stamp
	string frame_id
'''
def header_mutator(msg: dict):
  with mutator_context("header mutator"):
    new_msg = copy(msg)
    #TODO too may deepcopy calling other mutators
    new_msg['stamp'] = time_mutator(msg['stamp'])
    # skip frame_id field
  return new_msg

'''
bond/msg/Status
  std_msgs/Header header
  string id 
  string instance_id 
  bool active
  float32 heartbeat_timeout
  float32 heartbeat_period
'''
def status_mutator(msg: dict):
  with mutator_context("status mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    # skip id field
    # skip instance_id field
    new_msg['active'] = bmutator(msg['active'])
    new_msg['heartbeat_timeout'] = fmutator(msg['heartbeat_timeout'], 'float32')
    new_msg['heartbeat_period'] = fmutator(msg['heartbeat_period'], 'float32')
  return new_msg

'''
Vector3
	float64 x
	float64 y
	float64 z
'''
def vector3_mutator(msg: dict):
  with mutator_context("vector3 mutator"):
    new_msg = copy(msg)
    new_msg['x'] = fmutator(msg['x'], 'float64')
    new_msg['y'] = fmutator(msg['y'], 'float64')
    new_msg['z'] = fmutator(msg['z'], 'float64')
  return new_msg

'''
geometry_msgs/msg/Twist
  Vector3  linear
  Vector3  angular
'''
def twist_mutator(msg: dict):
  with mutator_context("twist mutator"):
    new_msg = copy(msg)
    new_msg['linear'] = vector3_mutator(msg['linear'])
    new_msg['angular'] = vector3_mutator(msg['angular'])
  return new_msg

'''
geometry_msgs/TwistWithCovariance
	Twist twist
	float64[36] covariance
'''
def twistwithcovariance_mutator(msg: dict):
  with mutator_context("twistwithcovariance mutator"):
    new_msg = copy(msg)
    new_msg['twist'] = twist_mutator(msg['twist'])
    new_msg['covariance'] = lmutator(msg['covariance'],
                                     lambda x:fmutator(x, 'float64'),
                                     fixed_size=True)
  return new_msg

'''
Point32
	float32 x
	float32 y
	float32 z
'''
def point32_mutator(msg: dict):
  with mutator_context("point32 mutator"):
    new_msg = copy(msg)
    new_msg['x'] = fmutator(msg['x'], 'float32')
    new_msg['y'] = fmutator(msg['y'], 'float32')
    new_msg['z'] = fmutator(msg['z'], 'float32')
  return new_msg

'''
Point64
	float64 x
	float64 y
	float64 z
'''
def point64_mutator(msg: dict):
  with mutator_context("point64 mutator"):
    new_msg = copy(msg)
    new_msg['x'] = fmutator(msg['x'], 'float64')
    new_msg['y'] = fmutator(msg['y'], 'float64')
    new_msg['z'] = fmutator(msg['z'], 'float64')
  return new_msg

'''
geometry_msgs/msg/Polygon
  Point32[] points
'''
def polygon_mutator(msg: dict):
  with mutator_context("polygon mutator"):
    new_msg = copy(msg)
    new_msg['points'] = lmutator(msg['points'], point32_mutator)
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
  with mutator_context("quaternion mutator"):
    new_msg = copy(msg)
    new_msg['x'] = fmutator(msg['x'], 'float64')
    new_msg['y'] = fmutator(msg['y'], 'float64')
    new_msg['z'] = fmutator(msg['z'], 'float64')
    new_msg['w'] = fmutator(msg['w'], 'float64')
  return new_msg

'''
geometry_msgs/Pose origin
  Point position
  Quaternion orientation
'''
def pose_mutator(msg: dict):
  with mutator_context("pose mutator"):
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
  with mutator_context("posestamped mutator"):
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
  with mutator_context("posewithcovariance mutator"):
    new_msg = copy(msg)
    new_msg['pose'] = pose_mutator(msg['pose'])
    new_msg['covariance'] = lmutator(msg['covariance'],
                                     lambda x: fmutator(x, 'float64'),
                                     fixed_size=True)
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
  with mutator_context("mapmetadata mutator"):
    new_msg = copy(msg)
    new_msg['map_load_time'] = time_mutator(msg['map_load_time'])
    new_msg['resolution'] = fmutator(msg['resolution'], 'float32')
    new_msg['width'] = fmutator(msg['width'], 'uint32')
    new_msg['height'] = fmutator(msg['height'], 'uint32')
    new_msg['origin'] = point32_mutator(msg['origin'])
  return new_msg

'''
nav_msgs/msg/OccupancyGrid
  std_msgs/Header header
  MapMetaData info
  int8[] data
'''
def occupancygrid_mutator(msg: dict):
  with mutator_context("occupancygrid mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    new_msg['info'] = mapmetadata_mutator(msg['info'])
    new_msg['data'] = lmutator(msg['data'], lambda x: fmutator(x, 'int8'))
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
  with mutator_context("costmapmetadata mutator"):
    new_msg = copy(msg)
    new_msg['map_load_time'] = time_mutator(msg['map_load_time'])
    new_msg['update_time'] = time_mutator(msg['update_time'])
    # skip layer field
    new_msg['resolution'] = fmutator(msg['resolution'], 'float32')
    new_msg['size_x'] = fmutator(msg['size_x'], 'uint32')
    new_msg['size_y'] = fmutator(msg['size_y'], 'uint32')
    new_msg['origin'] = pose_mutator(msg['origin'])
  return new_msg

'''
/local_costmap/costmap_raw, nav2_msgs/msg/Costmap
  std_msgs/Header header
  CostmapMetaData metadata
  uint8[] data
'''
def costmap_mutator(msg: dict):
  with mutator_context("costmap mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    new_msg['metadata'] = costmapmetadata_mutator(msg['metadata'])
    new_msg['data'] = lmutator(msg['data'], lambda x: fmutator(x, 'uint8'))
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
  with mutator_context("occupancygridupdate mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    new_msg['x'] = fmutator(msg['x'], 'int32')
    new_msg['y'] = fmutator(msg['y'], 'int32')
    new_msg['width'] = fmutator(msg['width'], 'uint32')
    new_msg['height'] = fmutator(msg['height'], 'uint32')
    new_msg['data'] = lmutator(msg['data'], lambda x: fmutator(x, 'int8'))
  return new_msg

'''
nav_msgs/msg/Path
  std_msgs/Header header
  geometry_msgs/PoseStamped[] poses
'''
def path_mutator(msg: dict):
  with mutator_context("path mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    new_msg['poses'] = lmutator(msg['poses'], posestamped_mutator)
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
  with mutator_context("laserscan mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    new_msg['angle_min'] = fmutator(msg['angle_min'], 'float32')
    new_msg['angle_max'] = fmutator(msg['angle_max'], 'float32')
    new_msg['angle_increment'] = fmutator(msg['angle_increment'], 'float32')
    new_msg['time_increment'] = fmutator(msg['time_increment'], 'float32')
    new_msg['scan_time'] = fmutator(msg['scan_time'], 'float32')
    new_msg['range_min'] = fmutator(msg['range_min'], 'float32')
    new_msg['range_max'] = fmutator(msg['range_max'], 'float32')
    new_msg['ranges'] = lmutator(msg['ranges'], lambda x: fmutator(x, 'float32'))
    new_msg['intensities'] = lmutator(msg['intensities'],
                                      lambda x: fmutator(x, 'float32'),)
  return new_msg

'''
Transform transform
  Vector3 translation
  Quaternion rotation
'''
def transform_mutator(msg: dict):
  with mutator_context("transform mutator"):
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
  with mutator_context("transformstamped mutator"):
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
  with mutator_context("odometry mutator"):
    new_msg = copy(msg)
    new_msg['header'] = header_mutator(msg['header'])
    # skip child_frame_id field
    new_msg['pose'] = posewithcovariance_mutator(msg['pose'])
    new_msg['twist'] = twistwithcovariance_mutator(msg['twist'])
  return new_msg