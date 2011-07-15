/** @file
 *
 * @brief ROS Memsense IMU generic driver (nodelet version)
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


#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "imu_base_node.h"


/** Nodelet interface for IMU driver.
 */
class IMUNodelet: public nodelet::Nodelet, private memsense_imu::IMUBaseNode
{
public:
  IMUNodelet();
private:
  virtual void onInit();
};


IMUNodelet::IMUNodelet()
: nodelet::Nodelet()
, memsense_imu::IMUBaseNode(ros::NodeHandle(getNodeHandle(),"imu"))
{}

/** Nodelet initialization.
 *  @note Must return immediately.
 */
void IMUNodelet::onInit()
{
  // Publishers and advertise topics
  advertiseTopics();

  // Node parameters and callback settings
  initDynParamsSrv();

  // Spin is done by nodelet machinery
}


// Register this plugin with pluginlib.
// Names must match *nodelet.xml in package root.
// Parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(memsense_imu, imu_nodelet, IMUNodelet, nodelet::Nodelet);
