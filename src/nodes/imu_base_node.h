/** @file
 *
 * @brief ROS Memsense IMU generic driver presentation.
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

#ifndef IMU_BASE_NODE_H
#define IMU_BASE_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <map>
#include "memsense_imu/IMUDynParamsConfig.h"
#include "IMUDataUtils/CommonUtils.h"
#include "IMUSampler/IMUSampler.h"
#include "imu_sample.h"
#include "imu_filter.h"

namespace memsense_imu
{

/** IMU Driver class
 *
 */
class IMUBaseNode
{
public:

  IMUBaseNode(const ros::NodeHandle& nh);

  /*---------------------------------------------------------------------------
  void initParams();
  ----------------------------------------------------------------------------*/

  void initDynParamsSrv();

  void advertiseTopics();

  void poll();
  
private:
  ros::NodeHandle node_;
  // ros::NodeHandle priv_;
  ros::Publisher pub_raw_;
  ros::Publisher pub_unbiased_;
  ros::Publisher pub_filtered_raw_;
  ros::Publisher pub_filtered_unbiased_;
  ros::Timer timed_caller_;
  dynamic_reconfigure::Server<memsense_imu::IMUDynParamsConfig> dyn_params_srv_;
  
  std::string frame_id_;
  
  Filter filter_;
  
  mems::IMUSampler sampler_;

  bool sampler_ready_;
  bool parser_ok_;
  bool port_ok_;
  
  std::string port_;
  mems::E_DeviceType imu_type_;
  double ranges_[NUM_MAGNS];
  VarianceTable vars_;
  BiasTable biases_;
  
  double filter_rate_;
  bool do_filtering_;

  void processData(const SampleArray& sample,
                   const BiasTable& bias,
                   const VarianceTable& var,
                   const ros::Publisher& pub_raw,
                   const ros::Publisher& pub_calibrated);
  
  void timedPublishCallback();
  
  void startFilterTimer();
  
  void stopFilterTimer();
  
  template <typename T>
  bool updateDynParam(T* param, const T& new_value) const;

  void dynReconfigureParams(memsense_imu::IMUDynParamsConfig& params, uint32_t level);
  
  // IMU types and names
  static const std::map<std::string,mems::E_DeviceType> IMU_TYPE_NAMES_;
  static std::map<std::string,mems::E_DeviceType> define_type_names();
  mems::E_DeviceType nameToDeviceType(const std::string& name) const;
};

}

#endif // IMU_BASE_NODE_H
