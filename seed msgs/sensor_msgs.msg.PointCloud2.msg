/mobile_base/sensors/bumper_pointcloud, sensor_msgs/msg/PointCloud2
/local_costmap/voxel_marked_cloud, sensor_msgs/msg/PointCloud2
/global_costmap/voxel_marked_cloud, sensor_msgs/msg/PointCloud2
#
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
uint32 height
uint32 width
PointField[] fields
	uint8 INT8    = 1
	uint8 UINT8   = 2
	uint8 INT16   = 3
	uint8 UINT16  = 4
	uint8 INT32   = 5
	uint8 UINT32  = 6
	uint8 FLOAT32 = 7
	uint8 FLOAT64 = 8
	string name      #
	uint32 offset    #
	uint8  datatype  #
	uint32 count     #
bool    is_bigendian #
uint32  point_step   #
uint32  row_step     #
uint8[] data         #
bool is_dense        #

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "overlapping_fields"
height: 1
width: 100
fields:
  - name: "x"
    offset: 0
    datatype: 7  # FLOAT32
    count: 1
  - name: "y"
    offset: 0  # Overlaps with x
    datatype: 7  # FLOAT32
    count: 1
is_bigendian: false
point_step: 8
row_step: 800
data: []
is_dense: true

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "excessive_steps"
height: 1
width: 10
fields:
  - name: "intensity"
    offset: 0
    datatype: 2  # UINT8
    count: 1
is_bigendian: false
point_step: 4294967295  # Max uint32
row_step: 4294967295    # Max uint32
data: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
is_dense: true

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "inconsistent_is_dense"
height: 1
width: 5
fields:
  - name: "z"
    offset: 0
    datatype: 7  # FLOAT32
    count: 1
is_bigendian: false
point_step: 4
row_step: 20
data: [NaN, NaN, NaN, NaN, NaN]  # Representing as NaN for illustration; actual storage might differ.
is_dense: true  # Inconsistent with the presence of NaN values.

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "zero_count_field"
height: 1
width: 100
fields:
  - name: "empty"
    offset: 0
    datatype: 7  # FLOAT32
    count: 0  # Zero count might confuse parsers or lead to division by zero errors.
is_bigendian: false
point_step: 4
row_step: 400
data: []
is_dense: false

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "large_count_small_step"
height: 1
width: 10
fields:
  - name: "x"
    offset: 0
    datatype: 7  # FLOAT32
    count: 100  # Large count value
is_bigendian: false
point_step: 4  # Smaller than necessary for the count of x
row_step: 40
data: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
is_dense: true

