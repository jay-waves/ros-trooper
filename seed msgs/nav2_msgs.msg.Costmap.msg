/local_costmap/costmap_raw, nav2_msgs/msg/Costmap
/global_costmap/costmap_raw, nav2_msgs/msg/Costmap

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
CostmapMetaData metadata
	builtin_interfaces/Time map_load_time
		int32 sec
		uint32 nanosec
	builtin_interfaces/Time update_time
		int32 sec
		uint32 nanosec
	string layer
	float32 resolution
	uint32 size_x
	uint32 size_y
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
uint8[] data