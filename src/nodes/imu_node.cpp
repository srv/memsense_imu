/** @file
 *
 * @brief ROS Memsense IMU generic driver (node version)
 *
 * @par Advertises
 *
 * - @b imu/data topic (sensor_msgs/Imu) IMU's raw data
 *
 * - @b imu/data_calibrated topic (sensor_msgs/Imu) Calibrated (bias removed)
 *   IMU data
 *
 * - @b imu/data_filtered topic (sensor_msgs/Imu) IMU's filtered output
 *  (mean value every sec seconds)
 *
 * - @b imu/data_filtered_calibrated topic (sensor_msgs/Imu) Calibrated (bias removed)
 *   filtered (mean every sec seconds) IMU data
 *
 * @par Parameters
 *
 * - @b ~imu_type Memsense device type (default nIMU_3temp)
 * 
 * - @b ~gyro_range gyroscop's range (default 150.0 degrees per second)
 * - @b ~accel_range accelerometer range (default 2.0 g's)
 * - @b ~mag_range magnetometer range (default 1.9 gauss)
 
 * - @b ~serial_port Serial port device file name (default /dev/ttyUSB0)
 
 * - @b ~gyro_var gyroscope's variance (default 0.0)
 * - @b ~accel_var accelerometer's variance (default 0.0)
 * - @b ~mag_var magnetometer's variance (default 0.0)
 * 
 * - @b ~gyro_bias_(x|y|z) gyroscope's bias in each axis (default 0.0)
 * - @b ~accel_bias_(x|y|z) accelerometer's bias in each axis (default 0.0)
 * - @b ~mag_bias_(x|y|z) magnetometer's bias in each axis (default 0.0)
 * 
 * - @b ~filter_rate filtered output rate (IMU samples in the interval are collected
 *   and the output is its mean)
 * 
 * - @b ~frame_id frame identifier for message header
 *
 */


#include "imu_base_node.h"

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "imu_node");

  ros::NodeHandle nh("imu");

  memsense_imu::IMUBaseNode imu_node(nh);

  // Publishers and advertise topics
  imu_node.advertiseTopics();
  
  // Node parameters
  imu_node.initDynParamsSrv();

  while(nh.ok())
  {
    try
    {
      imu_node.poll();
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM ("Unexpected error polling IMU: " << e.what());
    }
    ros::spinOnce();
  }

  return 0;
}
