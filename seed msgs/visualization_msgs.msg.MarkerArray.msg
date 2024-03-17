/waypoints, visualization_msgs/msg/MarkerArray
Marker[] markers
	#
	int32 ARROW=0
	int32 CUBE=1
	int32 SPHERE=2
	int32 CYLINDER=3
	int32 LINE_STRIP=4
	int32 LINE_LIST=5
	int32 CUBE_LIST=6
	int32 SPHERE_LIST=7
	int32 POINTS=8
	int32 TEXT_VIEW_FACING=9
	int32 MESH_RESOURCE=10
	int32 TRIANGLE_LIST=11
	int32 ADD=0
	int32 MODIFY=0
	int32 DELETE=2
	int32 DELETEALL=3
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string ns
	int32 id
	int32 type
	int32 action
	geometry_msgs/Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	geometry_msgs/Vector3 scale
		float64 x
		float64 y
		float64 z
	std_msgs/ColorRGBA color
		float32 r
		float32 g
		float32 b
		float32 a
	builtin_interfaces/Duration lifetime
		int32 sec
		uint32 nanosec
	bool frame_locked
	geometry_msgs/Point[] points
		float64 x
		float64 y
		float64 z
	std_msgs/ColorRGBA[] colors
		float32 r
		float32 g
		float32 b
		float32 a
	string texture_resource
	sensor_msgs/CompressedImage texture
		std_msgs/Header header #
			builtin_interfaces/Time stamp
				int32 sec
				uint32 nanosec
			string frame_id
		                             # Header frame_id should be optical frame of camera
		                             # origin of frame should be optical center of cameara
		                             # +x should point to the right in the image
		                             # +y should point down in the image
		                             # +z should point into to plane of the image
		string format                #
		                             #   Acceptable values:
		                             #     jpeg, png, tiff
		uint8[] data                 #
	UVCoordinate[] uv_coordinates
		float32 u
		float32 v
	string text
	string mesh_resource
	MeshFile mesh_file
		string filename
		uint8[] data
	bool mesh_use_embedded_materials
