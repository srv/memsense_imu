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

#ifndef IMU_NODE_BASE_H
#define IMU_NODE_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <memsense_imu/ImuMAG.h>
#include <dynamic_reconfigure/server.h>
#include <map>
#include "memsense_imu/IMUDynParamsConfig.h"
#include "IMUDataUtils/CommonUtils.h"
#include "IMUSampler/IMUSampler.h"
#include "imu_sample.h"
#include "imu_filter.h"

namespace memsense_imu
{

/**
 * @brief ROS interface class for IMU driver.
 */
class IMUNodeBase
{
public:

  IMUNodeBase(const ros::NodeHandle& node,
              const ros::NodeHandle& priv = ros::NodeHandle("~") );

  /*---------------------------------------------------------------------------
  void initParams();
  ----------------------------------------------------------------------------*/

  void initDynParamsSrv();

  void advertiseTopics();

  void poll();
  
  void outputFilter();

private:

  ros::NodeHandle node_; //!< IMU nampespace node handle
  ros::NodeHandle priv_; //!< Private nampespace node handle
  ros::Publisher pub_raw_;               //!< Raw standard IMU data publisher.
  ros::Publisher pub_unbiased_;          //!< Calibrated standard IMU data publisher.
  ros::Publisher pub_filtered_raw_;      //!< Filtered standard IMU data publisher.
  ros::Publisher pub_filtered_unbiased_; //!< Calibrated and filtered standard IMU data publisher.
  ros::Publisher pub_mag_;                   //!< Raw standard IMU data publisher.
  ros::Publisher pub_mag_unbiased_;          //!< Calibrated custom MAG data publisher.
  ros::Publisher pub_filtered_mag_;          //!< Filtered custom MAG data publisher.
  ros::Publisher pub_filtered_mag_unbiased_; //!< Calibrated and filtered custom MAG data publisher.
  ros::Timer polling_timer_; //!< Polling timer.
  ros::Timer filter_timer_;  //!< Output filter timer.
  dynamic_reconfigure::Server<memsense_imu::IMUDynParamsConfig> dyn_params_srv_; //!< Dynamic parameter server.
  
  std::string frame_id_; //!< IMU frame identifier.
  
  mems::IMUSampler sampler_; //!< IMU sampler.
  
  std::string port_;            //!< Serial port name.
  mems::E_DeviceType imu_type_; //!< Memsense IMU device type.
  double ranges_[NUM_MAGNS];    //!< Magnitude ranges (implicitly set the units).
  VarianceTable vars_;          //!< Magnitude variances.
  BiasTable biases_;            //!< Magnitude biases in each axis.
  
  bool sampler_ready_; //!< Sampler is ready when port is ok and parser is ok.
  bool parser_ok_;     //!< Sampler parser initialized.
  bool port_ok_;       //!< Sampler port opened.

  double polling_rate_; //!< Desired polling rate.

  Filter filter_; //!< IMU sample filter.

  double filter_rate_; //!< Output filter rate.
  bool do_filtering_;  //!< Filtering enabled.

  void outputData(const SampleArray& sample,
                  const BiasTable& bias,
                  const VarianceTable& var,
                  const ros::Time& stamp,
                  const std::string& frame_id,
                  const ros::Publisher& pub_raw,
                  const ros::Publisher& pub_calibrated);

  void outputMAGData(const SampleArray& sample,
                     const BiasTable& bias,
                     const VarianceTable& var,
                     const ros::Time& stamp,
                     const std::string& frame_id,
                     const ros::Publisher& pub_raw,
                     const ros::Publisher& pub_calibrated);

  template <typename T>
  bool updateDynParam(T* param, const T& new_value) const;

  void dynReconfigureParams(memsense_imu::IMUDynParamsConfig& params, uint32_t level);

  // IMU types and names
  static const std::map<std::string,mems::E_DeviceType> IMU_TYPE_NAMES_;
  static std::map<std::string,mems::E_DeviceType> define_type_names();
  mems::E_DeviceType nameToDeviceType(const std::string& name) const;
};

}

#endif // IMU_NODE_BASE_H
