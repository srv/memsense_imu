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


#include "imu_node_base.h"

/**
 * @brief Default empty constructor.
 * @param node IMU namespace node handle.
 * @param priv private namespace node handle.
 * @return
 */
memsense_imu::IMUNodeBase::IMUNodeBase(const ros::NodeHandle& node,
                                       const ros::NodeHandle& priv)
: node_(node),
  priv_(priv),
  dyn_params_srv_(priv),
  port_(""), imu_type_(mems::DT_INVALID),
  polling_rate_(0.0), filter_rate_(0.0)
{}

/**
 * @brief Initialize the dynamic parameter reconfiguring server.
 */
void memsense_imu::IMUNodeBase::initDynParamsSrv()
{
  dyn_params_srv_.setCallback( boost::bind(&IMUNodeBase::dynReconfigureParams,this,_1,_2) );
}

/**
 * @brief Advertise standard IMU and custom MAG data topics.
 */
void memsense_imu::IMUNodeBase::advertiseTopics()
{
  pub_raw_ = node_.advertise<sensor_msgs::Imu>("data",10);
  pub_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_calibrated",10);
  pub_filtered_raw_ = node_.advertise<sensor_msgs::Imu>("data_filtered",10);
  pub_filtered_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_filtered_calibrated",10);
  pub_mag_ = node_.advertise<sensor_msgs::MagneticField>("mag",10);
  pub_mag_unbiased_ = node_.advertise<sensor_msgs::MagneticField>("mag_calibrated",10);
  pub_filtered_mag_ = node_.advertise<sensor_msgs::MagneticField>("mag_filtered",10);
  pub_filtered_mag_unbiased_ = node_.advertise<sensor_msgs::MagneticField>("mag_filtered_calibrated",10);
}

/**
 * @brief Poll the unit for a new sample and publish it if success.
 *
 * If filtering is enabled, filter is updated too.
 * If the reading fails a warning is produced.
 */
void memsense_imu::IMUNodeBase::poll()
{
  try
  {
    if (sampler_ready_)
    {
      if( sampler_.readSample() )
      {
        ROS_DEBUG_STREAM("Sample read.");
        const ros::Time stamp = ros::Time::now();
        const std::string frame_id = frame_id_;
        SampleArray sample;
        sampler_.getDataReal(&sample[MAGN_GYRO], &sample[MAGN_ACCEL], &sample[MAGN_MAG]);
        if (do_filtering_)
          filter_.update(sample);
        outputData(sample, biases_, vars_, stamp, frame_id, pub_raw_, pub_unbiased_);
        outputMAGData(sample, biases_, vars_, stamp, frame_id, pub_mag_, pub_mag_unbiased_);
      }
      else
      {
        ROS_WARN_STREAM("Sample read failure.");
      }
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error polling IMU : " << e.what() );
  }
}

/**
 * @brief Publish raw and calibrated standard IMU data.
 * @param sample array of raw readings to output.
 * @param bias current bias values (for each magnitude and axis).
 * @param var current variance values (for each magnitude).
 * @param stamp sample time stamp.
 * @param frame_id IMU frame identifier.
 * @param pub_raw raw data publisher.
 * @param pub_calibrated calibrated data publisher.
 */
void memsense_imu::IMUNodeBase::outputData(const SampleArray& sample,
                                           const BiasTable& bias,
                                           const VarianceTable& var,
                                           const ros::Time& stamp,
                                           const std::string& frame_id,
                                           const ros::Publisher& pub_raw,
                                           const ros::Publisher& pub_calibrated)
{
  // messages to publish
  sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
  sensor_msgs::ImuPtr msg_unbias(new sensor_msgs::Imu());

  // fill header
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  
  // initiatize covariances
  for (int i=0; i<3; i++)
  {
    msg->orientation_covariance[4*i] = -1.0;
    msg->angular_velocity_covariance[4*i] = -1.0;
    msg->linear_acceleration_covariance[4*i] = -1.0;
    msg_unbias->orientation_covariance[4*i] = -1.0;
    msg_unbias->angular_velocity_covariance[4*i] = -1.0;
    msg_unbias->linear_acceleration_covariance[4*i] = -1.0;
  }
  
  // fill gyro values and covariances
  switch( sample[MAGN_GYRO].size() )
  {
    case 3 :
      msg->angular_velocity.z = sample[MAGN_GYRO][Z_AXIS];
      msg->angular_velocity_covariance[8] = var[MAGN_GYRO];
      msg_unbias->angular_velocity.z = sample[MAGN_GYRO][Z_AXIS]-bias[MAGN_GYRO][Z_AXIS];
      msg_unbias->angular_velocity_covariance[8] = var[MAGN_GYRO];
    case 2 :
      msg->angular_velocity.y = sample[MAGN_GYRO][Y_AXIS];
      msg->angular_velocity_covariance[4] = var[MAGN_GYRO];
      msg_unbias->angular_velocity.y = sample[MAGN_GYRO][Y_AXIS]-bias[MAGN_GYRO][Y_AXIS];
      msg_unbias->angular_velocity_covariance[4] = var[MAGN_GYRO];
    case 1 :
      msg->angular_velocity.x = sample[MAGN_GYRO][X_AXIS];
      msg->angular_velocity_covariance[0] = var[MAGN_GYRO];
      msg_unbias->angular_velocity.x = sample[MAGN_GYRO][X_AXIS]-bias[MAGN_GYRO][X_AXIS];
      msg_unbias->angular_velocity_covariance[0] = var[MAGN_GYRO];
  }
  
  // fill accel values and covariances
  switch( sample[MAGN_ACCEL].size() )
  {
    case 3 :
      msg->linear_acceleration.z = sample[MAGN_ACCEL][Z_AXIS];
      msg->linear_acceleration_covariance[8] = var[MAGN_ACCEL];
      msg_unbias->linear_acceleration.z = sample[MAGN_ACCEL][Z_AXIS]-bias[MAGN_ACCEL][Z_AXIS];
      msg_unbias->linear_acceleration_covariance[8] = var[MAGN_ACCEL];
    case 2 :
      msg->linear_acceleration.y = sample[MAGN_ACCEL][Y_AXIS];
      msg->linear_acceleration_covariance[4] = var[MAGN_ACCEL];
      msg_unbias->linear_acceleration.y = sample[MAGN_ACCEL][Y_AXIS]-bias[MAGN_ACCEL][Y_AXIS];
      msg_unbias->linear_acceleration_covariance[4] = var[MAGN_ACCEL];
    case 1 :
      msg->linear_acceleration.x = sample[MAGN_ACCEL][X_AXIS];
      msg->linear_acceleration_covariance[0] = var[MAGN_ACCEL];
      msg_unbias->linear_acceleration.x = sample[MAGN_ACCEL][X_AXIS]-bias[MAGN_ACCEL][X_AXIS];
      msg_unbias->linear_acceleration_covariance[0] = var[MAGN_ACCEL];
  }
  
  // publish;
  pub_raw.publish(msg);
  pub_calibrated.publish(msg_unbias);
}


/**
 * @brief Publish raw and calibrated custom MAG data.
 * @param sample array of raw readings to output.
 * @param bias current bias values (for each magnitude and axis).
 * @param var current variance values (for each magnitude).
 * @param stamp sample time stamp.
 * @param frame_id IMU frame identifier.
 * @param pub_raw raw data publisher.
 * @param pub_calibrated calibrated data publisher.
 */
void memsense_imu::IMUNodeBase::outputMAGData(const SampleArray& sample,
                                              const BiasTable& bias,
                                              const VarianceTable& var,
                                              const ros::Time& stamp,
                                              const std::string& frame_id,
                                              const ros::Publisher& pub_raw,
                                              const ros::Publisher& pub_calibrated)
{
  // messages to publish
  sensor_msgs::MagneticFieldPtr mag(new sensor_msgs::MagneticField());
  sensor_msgs::MagneticFieldPtr mag_unbias(new sensor_msgs::MagneticField());

  // fill header
  mag->header.stamp = stamp;
  mag->header.frame_id = frame_id;
  mag_unbias->header = mag->header;

  // initiatize covariances
  for (int i=0; i<3; i++)
  {
    mag->magnetic_field_covariance [4+i] = -1.0;
    mag_unbias->magnetic_field_covariance[4*i] = -1.0;
  }

  // fill magnet values and covariances
  switch( sample[MAGN_MAG].size() )
  {
    case 3 :
      mag->magnetic_field.z = sample[MAGN_MAG][Z_AXIS];
      mag->magnetic_field_covariance[8] = var[MAGN_MAG];
      mag_unbias->magnetic_field.z = sample[MAGN_MAG][Z_AXIS]-bias[MAGN_MAG][Z_AXIS];
      mag_unbias->magnetic_field_covariance[8] = var[MAGN_MAG];
    case 2 :
      mag->magnetic_field.y = sample[MAGN_MAG][Y_AXIS];
      mag->magnetic_field_covariance[4] = var[MAGN_MAG];
      mag_unbias->magnetic_field.y = sample[MAGN_MAG][Y_AXIS]-bias[MAGN_MAG][Y_AXIS];
      mag_unbias->magnetic_field_covariance[4] = var[MAGN_MAG];
    case 1 :
      mag->magnetic_field.x = sample[MAGN_MAG][X_AXIS];
      mag->magnetic_field_covariance[0] = var[MAGN_MAG];
      mag_unbias->magnetic_field.x = sample[MAGN_MAG][X_AXIS]-bias[MAGN_MAG][X_AXIS];
      mag_unbias->magnetic_field_covariance[0] = var[MAGN_MAG];
  }

  // publish;
  pub_raw.publish(mag);
  pub_calibrated.publish(mag_unbias);
}


/**
 * @brief Publish both standard IMU and custom MAG filtered data.
 */
void memsense_imu::IMUNodeBase::outputFilter()
{
  unsigned int count = filter_.count();
  if ( count != 0 )
  {
    SampleArray resp;
    const ros::Time stamp = ros::Time::now();
    const std::string frame_id = frame_id_;
    filter_.median(&resp);
    VarianceTable resp_vars;
    BiasTable resp_biases;
    for (int i=0; i<NUM_MAGNS; i++)
    {
      resp_vars[i] = vars_[i]/double(count);
      for (int j=0; j<NUM_AXES; j++)
        resp_biases[i][j] = biases_[i][j];
    }
    outputData(resp, resp_biases, resp_vars, stamp, frame_id, pub_filtered_raw_, pub_filtered_unbiased_);
    outputMAGData(resp, resp_biases, resp_vars, stamp, frame_id, pub_filtered_mag_, pub_filtered_mag_unbiased_);
    filter_.reset();
  }
}


/**
 * @brief Check for generic parameter update.
 * @param param parameter variable to be updated.
 * @param new_value new value for the parameter.
 * @return whether the parameter has been updated with a new value.
 */
template <typename T>
bool memsense_imu::IMUNodeBase::updateDynParam(T* param, const T& new_value) const
{
  if ( *param != new_value )
  {
    *param = new_value;
    return true;
  }
  else
    return false;
}

/**
 * @brief Reconfigure dynamic parameters callback.
 * @param params new parameter configuration.
 * @param level not used.
 */
void memsense_imu::IMUNodeBase::dynReconfigureParams(memsense_imu::IMUDynParamsConfig& params,
                                                     uint32_t level)
{
  try
  {
    updateDynParam(&vars_[MAGN_GYRO], params.gyros_var);
    updateDynParam(&vars_[MAGN_ACCEL], params.accels_var);
    updateDynParam(&vars_[MAGN_MAG], params.mags_var);

    updateDynParam(&biases_[MAGN_GYRO][X_AXIS], params.gyro_bias_x);
    updateDynParam(&biases_[MAGN_GYRO][Y_AXIS], params.gyro_bias_y);
    updateDynParam(&biases_[MAGN_GYRO][Z_AXIS], params.gyro_bias_z);

    updateDynParam(&biases_[MAGN_ACCEL][X_AXIS], params.accel_bias_x);
    updateDynParam(&biases_[MAGN_ACCEL][Y_AXIS], params.accel_bias_y);
    updateDynParam(&biases_[MAGN_ACCEL][Z_AXIS], params.accel_bias_z);

    updateDynParam(&biases_[MAGN_MAG][X_AXIS], params.mag_bias_x);
    updateDynParam(&biases_[MAGN_MAG][Y_AXIS], params.mag_bias_y);
    updateDynParam(&biases_[MAGN_MAG][Z_AXIS], params.mag_bias_z);

    updateDynParam(&frame_id_, params.frame_id);

    // all conditions must be evaluated to update the parameters
    // so sum them instead of || them
    if ( updateDynParam(&imu_type_, nameToDeviceType(params.imu_type)) +
         updateDynParam(&ranges_[MAGN_GYRO], params.gyros_range) +
         updateDynParam(&ranges_[MAGN_ACCEL], params.accels_range) +
         updateDynParam(&ranges_[MAGN_MAG], params.mags_range) )
    {
      parser_ok_ = false;
      // initialize sampler's parser
      ROS_INFO_STREAM("Initializing parser for device type '"
                      << params.imu_type << "'"
                      << " ( gyro range : " << ranges_[MAGN_GYRO]
                      << " , accel range : " << ranges_[MAGN_ACCEL]
                      << " , mag range : " << ranges_[MAGN_MAG] << " )");
      sampler_.initParser(imu_type_, ranges_[MAGN_GYRO],
                          ranges_[MAGN_ACCEL], ranges_[MAGN_MAG]);
      parser_ok_ = true;
    }

    if ( updateDynParam(&port_,params.serial_port) )
    {
      port_ok_ = false;
      // open comm port
      ROS_INFO_STREAM("Opening communication port on '" << port_ << "'");
      sampler_.openComm(port_);
      port_ok_ = true;
    }

    sampler_ready_ = parser_ok_ && port_ok_;

    if ( sampler_ready_ )
    {
      if (updateDynParam(&polling_rate_,params.polling_rate))
      {
        if ( polling_rate_==0.0 )
        {
          ROS_INFO_STREAM("Stopping raw output" );
          polling_timer_.stop();
        }
        else
        {
          ROS_INFO_STREAM("Setting polling rate to " << polling_rate_ << " hz");
          if (polling_timer_)
          {
            polling_timer_.setPeriod(ros::Duration(1.0/polling_rate_));
            polling_timer_.start();
          }
          else
          {
            polling_timer_ = node_.createTimer(ros::Duration(1.0/polling_rate_),
                                               boost::bind(&IMUNodeBase::poll,this));
          }
        }
      }
      if (updateDynParam(&filter_rate_,params.filter_rate))
      {
        if ( filter_rate_==0.0 )
        {
          ROS_INFO_STREAM("Stopping filtered output" );
          filter_timer_.stop();
          do_filtering_ = false;
        }
        else
        {
          ROS_INFO_STREAM("Setting filter rate to " << filter_rate_ << " hz");
          if (filter_timer_)
          {
            filter_timer_.setPeriod(ros::Duration(1.0/filter_rate_));
            filter_timer_.start();
          }
          else
          {
            filter_timer_ = node_.createTimer(ros::Duration(1.0/filter_rate_),
                                              boost::bind(&IMUNodeBase::outputFilter,this));
          }
          do_filtering_ = true;
        }
      }
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Error reconfiguring device : " << e.what());
    if (polling_rate_>0.0)
    {
      polling_timer_.stop();
      polling_rate_ = 0.0;
    }
    sampler_ready_ = false;
    if (filter_rate_>0.0)
    {
      filter_timer_.stop();
      filter_rate_ = 0.0;
    }
    do_filtering_ = true;
  }
}


/**
 * @brief Initialize the name string to Memsense IMU device type map.
 * @return map mapping each name string to the corresponding device type.
 */
std::map<std::string,mems::E_DeviceType>
memsense_imu::IMUNodeBase::define_type_names()
{
  std::map<std::string,mems::E_DeviceType> m;
  m["nIMU_3temp"] = mems::NIMU_3TEMP;
  return m;
}

const std::map<std::string,mems::E_DeviceType>
memsense_imu::IMUNodeBase::IMU_TYPE_NAMES_ = define_type_names();


/**
 * @brief Get the corresponding device type from the name string.
 * @param name string identifying a Memsense device type, as defined in its API.
 * @return corresponding device type or invalid type if name is not recognized.
 */
mems::E_DeviceType memsense_imu::IMUNodeBase::nameToDeviceType(const std::string& name) const
{
  std::map<std::string, mems::E_DeviceType>::const_iterator it = IMU_TYPE_NAMES_.find(name);
  if (it==IMU_TYPE_NAMES_.end())
    return mems::DT_INVALID;
  else
    return it->second;
}
