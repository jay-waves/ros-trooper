/map_updates, map_msgs/msg/OccupancyGridUpdate
/local_costmap/costmap_updates, map_msgs/msg/OccupancyGridUpdate
/global_costmap/costmap_updates, map_msgs/msg/OccupancyGridUpdate
/downsampled_costmap_updates, map_msgs/msg/OccupancyGridUpdate

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
int32 x
int32 y
uint32 width
uint32 height
int8[] data

header:
  stamp:
    sec: 2147483647
    nanosec: 999999999
  frame_id: "future_frame"
x: 0
y: 0
width: 100
height: 100
data: [0, -1, 127, -128, 0]

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "zero_dimensions"
x: 10
y: 10
width: 0
height: 0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "negative_coords"
x: -100
y: -100
width: 100
height: 100
data: [0, 1, 2, 3, 4]


header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "large_data"
x: 0
y: 0
width: 100
height: 100
data: [127, -128] * 5000


header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "mismatched_size"
x: 0
y: 0
width: 10
height: 10
data: [0, 1, 2, 3]  # Array size does not match width*height.

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "max_int_coords"
x: 2147483647
y: 2147483647
width: 100
height: 100
data: [0, 1, 2, 3, 4]

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "max_int_coords"
x: 2147483647
y: 2147483647
width: 100
height: 100
data: [0, 1, 2, 3, 4]

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "repeated_value_data"
x: 0
y: 0
width: 10
height: 10
data: [0] * 100  # Single value repeated to fill the array.

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "intense_values"
x: 0
y: 0
width: 2
height: 2
data: [127, -128, 127, -128]

header:
  stamp:
    sec: 158834234
    nanosec: 0
  frame_id: "zero_nanosec"
x: 0
y: 0
width: 100
height: 100
data: [0, 1, 2, 3, 4]

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "high_dimensions_min_data"
x: 0
y: 0
width: 4294967295
height: 4294967295
data: [0]  # Minimal data for huge dimensions.

