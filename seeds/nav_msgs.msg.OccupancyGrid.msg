/map, nav_msgs/msg/OccupancyGrid
/local_costmap/costmap, nav_msgs/msg/OccupancyGrid
/global_costmap/costmap, nav_msgs/msg/OccupancyGrid
/downsampled_costmap, nav_msgs/msg/OccupancyGrid

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
MapMetaData info
	builtin_interfaces/Time map_load_time
		int32 sec
		uint32 nanosec
	float32 resolution
	uint32 width
	uint32 height
	geometry_msgs/Pose origin
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
int8[] data

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 0  # Zero width might reveal handling issues for map size assumptions.
  height: 0  # Zero height, similarly problematic.
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []


header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "high_res_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1e-9  # Extremely high resolution might cause precision or overflow issues.
  width: 1000
  height: 1000
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "large_map_min_data"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 0.5
  width: 4294967295  # Max uint32, tests handling of large map dimensions.
  height: 4294967295
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: [0]

header:
  stamp:
    sec: -1  # Negative timestamp, tests error handling for time travel scenarios.
    nanosec: 0
  frame_id: "neg_time_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 100
  height: 100
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "extreme_origin_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 100
  height: 100
  origin:
    position:
      x: 1.7976931348623157e+308
      y: 1.7976931348623157e+308
      z: 1.7976931348623157e+308  # Extreme origin coordinates might expose handling issues for large values.
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "non_normalized_quaternion"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 100
  height: 100
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 1.0
      y: 1.0
      z: 1.0
      w: 1.0  # Non-normalized quaternion might reveal logic bugs in orientation handling.
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "future_load_time_map"
info:
  map_load_time:
    sec: 214748  # Far future load time, tests handling of future-dated resources.
    nanosec: 123
  resolution: 1.0
  width: 100
  height: 100
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "non_uniform_data_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 10
  height: 10
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: [0, 127, -128, 0, 127, -128, 0, 127, -128, 0]  # Non-uniform data, potentially exposing issues in data parsing or map rendering logic.

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "high_precision_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.175494351e-38  # Near float32 precision limit, might expose handling or rounding errors.
  width: 100
  height: 100
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []

header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: "empty_data_map"
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 1.0
  width: 100
  height: 100
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: []  # Empty data array but with specified non-zero dimensions, might reveal logic bugs in data handling or assumptions.
