/** @file
 *
 * @brief ROS Memsense IMU generic driver
 *
 * This is a ROS generic driver for inertial measurement units (IMUs)
 * provided by Memsense. It uses Memsense' libraries to handle IMU's output
 * on the serial port (with some errors fixed, and a rewritten serial port
 * class for Unix alike systems).
 * Output units are the same in which the range is specified.
 * The correct ranges for the device and its biases and variances should
 * be set.
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
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include "memsense_imu/IMUDynParamsConfig.h"
#include "IMUDataUtils/CommonUtils.h"
#include "IMUSampler/IMUSampler.h"

const int NUM_IMU_TYPES = 1;
const struct {std::string name; mems::E_DeviceType type;} IMU_TYPE_NAMES[NUM_IMU_TYPES] =
{
  {"nIMU_3temp" , mems::NIMU_3TEMP}
};

mems::E_DeviceType nameToDeviceType(const std::string& name)
{
  for (int i=0; i<NUM_IMU_TYPES; i++)
    if (name == IMU_TYPE_NAMES[i].name)
      return IMU_TYPE_NAMES[i].type;
  return mems::DT_INVALID;
}

enum Magnitude { MAGN_GYRO=0, MAGN_ACCEL=1, MAGN_MAG=2 };
const int NUM_MAGNS = 3;

enum Axe {X_AXIS=0, Y_AXIS=1, Z_AXIS=2};
const int NUM_AXES = 3;

class Filter
{
public:

  Filter()
  : count_(0)
  {
    for (int i=0; i<NUM_MAGNS; i++)
      for(int j=0; j<NUM_AXES; j++)
        sample_[i][j] = 0.0;
  }

  void update(const std::vector<double>& gyro,
              const std::vector<double>& accel,
              const std::vector<double>& mag)
  {
      for(int i=gyro.size()-1; i>=0; i--)
        sample_[MAGN_GYRO][i] += gyro[i];
      for(int i=accel.size()-1; i>=0; i--)
        sample_[MAGN_ACCEL][i] += accel[i];
      for(int i=mag.size()-1; i>=0; i--)
        sample_[MAGN_MAG][i] += mag[i];
      count_++;
  }

  void mean(std::vector<double>* gyro_means,
            std::vector<double>* accel_means,
            std::vector<double>* mag_means)
  {
    double n = count_;
    std::vector<double>* output[NUM_MAGNS];
    output[MAGN_GYRO] = gyro_means;
    output[MAGN_ACCEL] = accel_means;
    output[MAGN_MAG] = mag_means;
    for (int magn=0; magn<3; magn++)
      if(output[magn])
      {
        output[magn]->clear();
        for (int axe=0; axe<NUM_AXES; axe++)
          output[magn]->push_back(sample_[magn][axe]/n) ;
      }
  }

  void reset()
  {
    for (int i=0; i<NUM_MAGNS; i++)
      for(int j=0; j<NUM_AXES; j++)
        sample_[i][j] = 0.0;
    count_ = 0;
  }

  unsigned int count()
  {
    return count_;
  }

private:

  double sample_[NUM_MAGNS][NUM_AXES];
  unsigned int count_;

};


class IMUNode
{
public:

  typedef std::vector<double> SampleArray[NUM_MAGNS];
  typedef double VarianceArray[NUM_MAGNS];

  IMUNode()
  : node_("imu"), // priv_("~"),
    sampler_ready_(false), parser_ok_(false), port_ok_(false),
    imu_type_(mems::DT_INVALID), 
    filter_rate_(-1.0), do_filtering_(false)
  {
    for (int i=0; i<NUM_MAGNS; i++)
    {
      ranges_[i] = -1.0;
      vars_[i] = -1.0;
    }
  }

  /*---------------------------------------------------------------------------
  void initParams()
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

  void initDynParamsSrv()
  {
    dyn_params_srv_.setCallback( boost::bind(&IMUNode::dynReconfigureParams,this,_1,_2) );
  }

  void advertiseTopics()
  {
    pub_raw_ = node_.advertise<sensor_msgs::Imu>("data",1);
    pub_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_calibrated",1);
    pub_filtered_raw_ = node_.advertise<sensor_msgs::Imu>("data_filtered",1);
    pub_filtered_unbiased_ = node_.advertise<sensor_msgs::Imu>("data_filtered_calibrated",1);
  }

  void spin()
  {
    SampleArray sample;
    // spin
    while (node_.ok())
    {
      try
      {
        if (sampler_ready_)
        {
          if( sampler_.readSample() )
          {
            ROS_DEBUG_STREAM("Sample read");
            sampler_.getDataReal(&sample[MAGN_GYRO], &sample[MAGN_ACCEL], &sample[MAGN_MAG]);
            if (do_filtering_)
              filter_.update(sample[MAGN_GYRO], sample[MAGN_ACCEL], sample[MAGN_MAG]);
            processData(pub_raw_, pub_unbiased_, sample, vars_);
          }
          else
          {
            ROS_DEBUG_STREAM("Sample read failure");
          }
        }
        ros::spinOnce();
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM ("[imu_node] Unexpected error : " << e.what());
      }
    }
  }

private:
  ros::NodeHandle node_;
  // ros::NodeHandle priv_;
  ros::Publisher pub_raw_;
  ros::Publisher pub_unbiased_;
  ros::Publisher pub_filtered_raw_;
  ros::Publisher pub_filtered_unbiased_;
  sensor_msgs::Imu msg_;
  sensor_msgs::Imu msg_filtered_;
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
  double vars_[NUM_MAGNS];
  double biases_[NUM_MAGNS][NUM_AXES];
  
  double filter_rate_;
  bool do_filtering_;


  inline
  void initImuMsg(sensor_msgs::Imu* msg)
  {
    // initialize sample msg covariance matrices to -I
    msg->angular_velocity_covariance.assign(0.0);
    msg->linear_acceleration_covariance.assign(0.0);
    msg->orientation_covariance.assign(0.0);
    for (int i=0; i<3; i++)
    {
      msg->angular_velocity_covariance[4*i] = -1.0;
      msg->linear_acceleration_covariance[4*i] = -1.0;
      msg->orientation_covariance[4*i] = -1.0;
    }
  }
  
  inline 
  void resetImuMsg(sensor_msgs::Imu* msg)
  {
    // reset sample msg covariance matrices to -I
    for (int i=0; i<3; i++)
    {
      msg->angular_velocity_covariance[4*i] = -1.0;
      msg->linear_acceleration_covariance[4*i] = -1.0;
      msg->orientation_covariance[4*i] = -1.0;
    }
  }
  
  inline 
  void fillImuMsg(sensor_msgs::Imu* msg, SampleArray sample, VarianceArray var)
  {
    // fill gyro values and covariances
    switch( sample[MAGN_GYRO].size() )
    {
      case 3 :
        msg->angular_velocity.z = sample[MAGN_GYRO][2];
        msg->angular_velocity_covariance[8] = var[MAGN_GYRO];
      case 2 :
        msg->angular_velocity.y = sample[MAGN_GYRO][1];
        msg->angular_velocity_covariance[4] = var[MAGN_GYRO];
      case 1 :
        msg->angular_velocity.x = sample[MAGN_GYRO][0];
        msg->angular_velocity_covariance[0] = var[MAGN_GYRO];
    }
    // fill accel values and covariances
    switch( sample[MAGN_ACCEL].size() )
    {
      case 3 :
        msg->linear_acceleration.z = sample[MAGN_ACCEL][2];
        msg->linear_acceleration_covariance[8] = var[MAGN_ACCEL];
      case 2 :
        msg->linear_acceleration.y = sample[MAGN_ACCEL][1];
        msg->linear_acceleration_covariance[4] = var[MAGN_ACCEL];
      case 1 :
        msg->linear_acceleration.x = sample[MAGN_ACCEL][0];
        msg->linear_acceleration_covariance[0] = var[MAGN_ACCEL];
    }
  }
  
  inline
  void unbiasImuMsg(sensor_msgs::Imu* msg)
  {
    msg->angular_velocity.x -= biases_[MAGN_GYRO][X_AXIS];
    msg->angular_velocity.y -= biases_[MAGN_GYRO][Y_AXIS];
    msg->angular_velocity.z -= biases_[MAGN_GYRO][Z_AXIS];
    msg->linear_acceleration.x -= biases_[MAGN_ACCEL][X_AXIS];
    msg->linear_acceleration.y -= biases_[MAGN_ACCEL][Y_AXIS];
    msg->linear_acceleration.z -= biases_[MAGN_ACCEL][Z_AXIS];
  }

  void processData(const ros::Publisher& pub_raw,
                   const ros::Publisher& pub_calibrated,
                   SampleArray sample,
                   VarianceArray var)
  {
    // fill header
    msg_.header.stamp = ros::Time::now();
    msg_.header.frame_id = frame_id_;
    
    // reset covariances
    resetImuMsg(&msg_);
    
    fillImuMsg(&msg_, sample, var);
    pub_raw.publish(msg_);
    unbiasImuMsg(&msg_);
    pub_calibrated.publish(msg_);
  }

  void timedPublishCallback()
  {
    double count = filter_.count();
    if ( count != 0.0 )
    {
      SampleArray means;
      filter_.mean(&means[MAGN_GYRO],&means[MAGN_ACCEL],&means[MAGN_MAG]);
      VarianceArray mean_vars;
      for (int i=0; i<NUM_MAGNS; i++)
      {
        mean_vars[i] = vars_[i]/count;
      }
      processData(pub_filtered_raw_, pub_filtered_unbiased_, means, mean_vars);
      filter_.reset();
    }
  }
  
  void startFilterTimer()
  {
    if (timed_caller_)
    {
      timed_caller_.setPeriod(ros::Duration(1.0/filter_rate_));
      timed_caller_.start();
    }
    else
    {
      timed_caller_ = node_.createTimer(ros::Duration(1.0/filter_rate_),
                            boost::bind(&IMUNode::timedPublishCallback,this));
    }
  }
  
  void stopFilterTimer()
  {
    if (timed_caller_)
    {
      timed_caller_.stop();
    }
  }
  
  template <typename T>
  bool updateDynParam(T* param, const T& new_value)
  {
    if ( *param != new_value )
    {
      *param = new_value;
      return true;
    }
    else
      return false;
  }

  void dynReconfigureParams(memsense_imu::IMUDynParamsConfig& params, uint32_t level)
  {
    try
    {
      if ( updateDynParam(&vars_[MAGN_GYRO], params.gyros_var) +
           updateDynParam(&vars_[MAGN_ACCEL], params.accels_var) +
           updateDynParam(&vars_[MAGN_MAG], params.mags_var) )
        // update imu msg due to change in variances
        initImuMsg(&msg_);

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
      
      sampler_ready_ = parser_ok_ && port_ok_ ;
      
      if ( sampler_ready_ )
      {
        if (updateDynParam(&filter_rate_,params.filter_rate))
        {
          if ( filter_rate_>0.0 )
          {
            ROS_INFO_STREAM("Setting filtered output rate to " << filter_rate_);
            startFilterTimer();
            do_filtering_ = true;
          }
          else
          {
            ROS_INFO_STREAM("Stopping filtered output" );
            stopFilterTimer();
            do_filtering_ = false;
          }
        }
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("[imu_node] Error reconfiguring device : " << e.what());
      stopFilterTimer();
      filter_rate_ = -1.0;
      sampler_ready_ = false;
    }
  }

};

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "imu_node");
  IMUNode imu_node;

  // Publishers and advertise topics
  imu_node.advertiseTopics();
  
  // Node parameters
  imu_node.initDynParamsSrv();

  imu_node.spin();

  return 0;
}
