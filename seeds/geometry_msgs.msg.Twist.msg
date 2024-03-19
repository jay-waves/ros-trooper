/cmd_vel, geometry_msgs/msg/Twist
/cmd_vel_nav, geometry_msgs/msg/Twist
/cmd_vel_teleop, geometry_msgs/msg/Twist

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

linear:
  x: 1e-323
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0

linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 1e+308
  y: 1e-16
  z: 1e-16

linear:
  x: 2.220446049250313e-16  # Machine epsilon for double precision floating-point
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0

linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 1.0000000000000002  # Slightly over one, testing for rounding or precision issues.
  y: 0.0
  z: 0.0

linear:
  x: 1.7976931348623157e+308
  y: 1.7976931348623157e+308
  z: 1.7976931348623157e+308
angular:
  x: 0.0
  y: 0.0
  z: 0.0

linear:
  x: 1.7976931348623157e+308
  y: 1.7976931348623157e+308
  z: 1.7976931348623157e+308
angular:
  x: 0.0
  y: 0.0
  z: 0.0

linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 1e-323
  z: 1e-323

linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: -1e+308
  y: 2.220446049250313e-16
  z: 1e+308

Vector3 linear
    float64 x = 3.0
    float64 y = 4.0
    float64 z = 0.0
Vector3 angular
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 5.0

Vector3 linear
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0
Vector3 angular
    float64 x = 100.0
    float64 y = 10.0
    float64 z = 1.0

Vector3 linear
    float64 x = -1.0
    float64 y = 1.0
    float64 z = -1.0
Vector3 angular
    float64 x = 1.0
    float64 y = -1.0
    float64 z = 1.0




Vector3 linear
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 1.7976931348623157e+308  # Maximum positive value for float64, tests handling of extreme magnitudes.
Vector3 angular
    float64 x = -1.7976931348623157e+308  # Maximum negative value for float64, tests handling of extreme negative magnitudes.
    float64 y = 0.0
    float64 z = 0.0

Vector3 linear
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0
Vector3 angular
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0

Vector3 linear
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0
Vector3 angular
    float64 x = -0.0  # Negative zero
    float64 y = -0.0  # Negative zero
    float64 z = -0.0  # Negative zero

Vector3 linear
    float64 x = 1.7976931348623157e+308
    float64 y = 1.7976931348623157e+308
    float64 z = 1.7976931348623157e+308
Vector3 angular
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0

Vector3 linear
    float64 x = 0.0
    float64 y = 0.0
    float64 z = 0.0
Vector3 angular
    float64 x = -1.7976931348623157e+308
    float64 y = 1.7976931348623157e+308
    float64 z = -1.7976931348623157e+308

