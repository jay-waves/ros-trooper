/scan, sensor_msgs/msg/LaserScan
#
std_msgs/Header header #
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis
float32 angle_min            #
float32 angle_max            #
float32 angle_increment      #
float32 time_increment       # time between measurements [seconds] - if your scann
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            #
float32 range_min            #
float32 range_max            #
float32[] ranges             #
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If yo
                             # device does not provide intensities, please leave
                             # the array empty.
