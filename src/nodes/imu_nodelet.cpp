/**
 * @file
 * @brief ROS Memsense IMU generic driver implementation.
 *
 * This is a ROS generic driver for inertial measurement units (IMUs)
 * provided by Memsense. It uses Memsense' libraries to handle IMU's output
 * on the serial port (with some errors fixed, and a rewritten serial port
 * class for Unix alike systems).
 * Output units are the same in which the range is specified.
 * The correct ranges for the device and its biases and variances should
 * be set.
 * A filter is implemented in the driver too.
 *
 * @par Advertises
 *
 * - @b imu/data topic (sensor_msgs/Imu) IMU's raw data.
 *
 * - @b imu/data_calibrated topic (sensor_msgs/Imu) Calibrated (bias removed)
 *   IMU data.
 *
 * - @b imu/data_filtered topic (sensor_msgs/Imu) IMU's filtered output
 *  (mean value every sec seconds).
 *
 * - @b imu/data_filtered_calibrated topic (sensor_msgs/Imu) Calibrated (bias removed)
 *   filtered (mean every sec seconds) IMU data.
 *
 * - @b imu/mag topic (memsense_imu/ImuMAG) IMU's raw data with magnetic field.
 *
 * - @b imu/mag_calibrated topic (memsense_imu/ImuMAG) Calibrated (bias removed)
 *   IMU data with magnetic field.
 *
 * - @b imu/mag_filtered topic (memsense_imu/ImuMAG) IMU's filtered output
 *   with magnetic field (mean value every sec seconds).
 *
 * - @b imu/mag_filtered_calibrated topic (memsense_imu/ImuMAG) Calibrated (bias removed)
 *   filtered (mean every sec seconds) IMU data with magnetic field.
 *
 * @par Parameters
 *
 * - @b ~imu_type Memsense device type (default nIMU_3temp).
 *
 * - @b ~gyro_range gyroscop's range (default 150.0 degrees per second).
 * - @b ~accel_range accelerometer range (default 2.0 g's).
 * - @b ~mag_range magnetometer range (default 1.9 gauss).

 * - @b ~serial_port Serial port device file name (default /dev/ttyUSB0).

 * - @b ~gyro_var gyroscope's variance (default 0.0).
 * - @b ~accel_var accelerometer's variance (default 0.0).
 * - @b ~mag_var magnetometer's variance (default 0.0).
 *
 * - @b ~gyro_bias_(x|y|z) gyroscope's bias in each axis (default 0.0).
 * - @b ~accel_bias_(x|y|z) accelerometer's bias in each axis (default 0.0).
 * - @b ~mag_bias_(x|y|z) magnetometer's bias in each axis (default 0.0).
 *
 * - @b ~filter_rate filtered output rate (IMU samples in the interval
 *      are collected and the output is its mean).
 *
 * - @b ~frame_id frame identifier for message header.
 */


#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "imu_node_base.h"

namespace memsense_imu
{

/**
 * @brief Nodelet interface for IMU driver.
 */
class IMUNodelet: public nodelet::Nodelet
{
public:
  IMUNodelet();
private:
  virtual void onInit();
  boost::shared_ptr<memsense_imu::IMUNodeBase> imu_node_;
};

} // namespace

/**
 * @brief Default constructor (doing nothing).
 * @return
 */
memsense_imu::IMUNodelet::IMUNodelet()
{}

/**
 * @brief Nodelet initialization.
 * @note Must return immediately.
 */
void memsense_imu::IMUNodelet::onInit()
{
  ros::NodeHandle node(getNodeHandle(),"imu");
  ros::NodeHandle priv(getPrivateNodeHandle());
  imu_node_.reset(new memsense_imu::IMUNodeBase(node,priv));

  // Publishers and advertise topics
  imu_node_->advertiseTopics();

  // Node parameters and callback settings
  imu_node_->initDynParamsSrv();

  // Spin is done by nodelet machinery
}


// Register this plugin with pluginlib.
// Names must match *nodelet.xml in package root.
// Parameters are: package, class name, class type, base class type
// PLUGINLIB_DECLARE_CLASS(memsense_imu, imu_nodelet, memsense_imu::IMUNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(memsense_imu::IMUNodelet, nodelet::Nodelet);
