/** @file
 *
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

memsense_imu::IMUBaseNode::IMUBaseNode(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& priv)
: node_(nh),
  priv_(priv),
  dyn_params_srv_(priv),
  port_(""), imu_type_(mems::DT_INVALID),
  polling_rate_(0.0), filter_rate_(0.0)
{}

/*---------------------------------------------------------------------------
void memsense_imu::IMUBaseNode::initParams()
{
  //TODO add a parameter for the imu_type_ and frame_id
  frame_id_ = "imu";
  imu_type_ = mems::NIMU_3TEMP;

  const std::string MAGN_NAMES[NUM_MAGNS] = {"gyro","accel","mag"};
  const double default_ranges[NUM_MAGNS] = {150.0,2.0,1.9};
  const double default_vars[NUM_MAGNS] = {0.0,0.0,0.0};
  const double default_bias = 0.0;
  for (int i=0; i<NUM_MAGNS; i++)
  {
    priv_.param(MAGN_NAMES[i]+"_range",ranges_[i],default_ranges[i]);
    priv_.param(MAGN_NAMES[i]+"_var",vars_[i],default_vars[i]);
    priv_.param(MAGN_NAMES[i]+"_bias_x",biases_[i][X_AXIS],default_bias);
    priv_.param(MAGN_NAMES[i]+"_bias_y",biases_[i][Y_AXIS],default_bias);
    priv_.param(MAGN_NAMES[i]+"_bias_z",biases_[i][Z_AXIS],default_bias);
  }
  priv_.param<std::string>("port",port_,"/dev/ttyUSB0");
  priv_.param("sec",sec_,0.666); // 0.6 should be equal to every 100 msgs
}
----------------------------------------------------------------------------*/

void memsense_imu::IMUBaseNode::initDynParamsSrv()
{
  dyn_params_srv_.setCallback( boost::bind(&IMUBaseNode::dynReconfigureParams,this,_1,_2) );
}

void memsense_imu::IMUBaseNode::advertiseTopics()
{
  pub_raw_ = node_.advertise<sensor_msgs::Imu>("data",1);
  pub_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_calibrated",1);
  pub_filtered_raw_ = node_.advertise<sensor_msgs::Imu>("data_filtered",1);
  pub_filtered_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_filtered_calibrated",1);
}

void memsense_imu::IMUBaseNode::poll()
{
  try
  {
    if (sampler_ready_)
    {
      if( sampler_.readSample() )
      {
        ROS_DEBUG_STREAM("Sample read.");
        SampleArray sample;
        sampler_.getDataReal(&sample[MAGN_GYRO], &sample[MAGN_ACCEL], &sample[MAGN_MAG]);
        if (do_filtering_)
          filter_.update(sample);
        processData(sample, biases_, vars_, pub_raw_, pub_unbiased_);
      }
      else
      {
        ROS_DEBUG_STREAM("Sample read failure.");
      }
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error polling IMU : " << e.what() );
  }
}

void memsense_imu::IMUBaseNode::processData(const SampleArray& sample,
                                            const BiasTable& bias,
                                            const VarianceTable& var,
                                            const ros::Publisher& pub_raw,
                                            const ros::Publisher& pub_calibrated)
{
  // messages to publish
  sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
  sensor_msgs::ImuPtr msg_unbias(new sensor_msgs::Imu());

  // fill header
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg_unbias->header = msg->header;
  
  // initiatize covariances
  for (int i=0; i<3; i++)
  {
    msg->orientation_covariance[4*i] = -1.0;
    msg_unbias->orientation_covariance[4*i] = -1.0;
    msg->angular_velocity_covariance[4*i] = -1.0;
    msg->linear_acceleration_covariance[4*i] = -1.0;
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
  
void memsense_imu::IMUBaseNode::outputFilter()
{
  unsigned int count = filter_.count();
  if ( count != 0 )
  {
    SampleArray resp;
    filter_.median(&resp);
    VarianceTable resp_vars;
    BiasTable resp_biases;
    for (int i=0; i<NUM_MAGNS; i++)
    {
      resp_vars[i] = vars_[i]/double(count);
      for (int j=0; j<NUM_AXES; j++)
        resp_biases[i][j] = biases_[i][j];
    }
    processData(resp, resp_biases, resp_vars, pub_filtered_raw_, pub_filtered_unbiased_);
    filter_.reset();
  }
}

template <typename T>
bool memsense_imu::IMUBaseNode::updateDynParam(T* param, const T& new_value) const
{
  if ( *param != new_value )
  {
    *param = new_value;
    return true;
  }
  else
    return false;
}

void memsense_imu::IMUBaseNode::dynReconfigureParams(memsense_imu::IMUDynParamsConfig& params,
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
                                               boost::bind(&IMUBaseNode::poll,this));
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
                                              boost::bind(&IMUBaseNode::outputFilter,this));
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

std::map<std::string,mems::E_DeviceType>
memsense_imu::IMUBaseNode::define_type_names()
{
  std::map<std::string,mems::E_DeviceType> m;
  m["nIMU_3temp"] = mems::NIMU_3TEMP;
  return m;
}

const std::map<std::string,mems::E_DeviceType>
memsense_imu::IMUBaseNode::IMU_TYPE_NAMES_ = define_type_names();

mems::E_DeviceType memsense_imu::IMUBaseNode::nameToDeviceType(const std::string& name) const
{
  std::map<std::string, mems::E_DeviceType>::const_iterator it = IMU_TYPE_NAMES_.find(name);
  if (it==IMU_TYPE_NAMES_.end())
    return mems::DT_INVALID;
  else
    return it->second;
}
