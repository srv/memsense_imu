# Version of sensor_msgs/Imu message 
# with added magnetometer and temperature readings.

Header header

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z

geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance # Row major x, y z
