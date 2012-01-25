/**
 * @file
 * @brief Sampler for the Memsense family of IMUs, presentation.
 * @author Joan Pau Beltran
 * @date 2011-02-08
 *
 * This file presents an IMU sampler class for the family of such devices
 * provided by Memsense. It uses the libraries provided by Memsense, with
 * some modifications to run on GNU/Linux (and probably other UNIX-alike
 * systems).
 */

#ifndef IMUSAMPLER_H
#define IMUSAMPLER_H

#include "IMUDataUtils/Types.h" // mems types such as data formats or device types
#include "IMUDataUtils/SerialComm.h"     // serial class (RS422 port)
#include "IMUDataUtils/STDIMUSample.h"   // general sample parser for all IMUs
#include <stdexcept>


namespace mems
{

/**
 * @brief Exception to handle IMU errors and exceptions.
 */
class IMUError : public std::runtime_error
{
public:
  IMUError(const std::string& msg) :
    std::runtime_error(msg)
  {
  }
};

/**
 * @brief IMU sampler class for the Memsense IMU family.
 */
class IMUSampler
{
public:

  IMUSampler();
  ~IMUSampler();

  void initParser(E_DeviceType device_type, double gyro_range, double accel_range,
                  double mag_range, double hig_range=0.0);
  void deleteParser();
  std::string getDeviceName() const;
  void setFormat(E_DataFormatType format);
  E_DataFormatType getFormat() const;

  void openComm(std::string device_name, unsigned long baud_rate=115200);
  void closeComm();
  std::string getCommName() const;
  
  bool readSample();
  
  void getDataReal(std::vector<double>* gyro,
                   std::vector<double>* accel,
                   std::vector<double>* mag,
                   std::vector<double>* temp = 0,
                   std::vector<double>* hig = 0);
  void getDataCounts(std::vector<short>* gyro,
                     std::vector<short>* accel,
                     std::vector<short>* mag,
                     std::vector<short>* temp = 0,
                     std::vector<short>* hig = 0);
  void getDataBytes(std::vector<unsigned char>* gyro,
                    std::vector<unsigned char>* accel,
                    std::vector<unsigned char>* mag,
                    std::vector<unsigned char>* temp = 0,
                    std::vector<unsigned char>* hig = 0);

private:

  // configuration data for device
  E_DeviceType device_type_; // type of device
  std::string device_type_name_; // device type's name
  int num_hig_axes_; // number of hi-g axes
  int num_temp_sensors_; // number of temperature sensors
  double gyro_range_; // range for gyro
  double accel_range_; // range for accel
  double mag_range_; // range for mag
  double hig_range_; // range for hi-g accel (uimu)

  // output format/units
  E_DataFormatType data_format_; // type of data format to output
  STDIMUSample * parser_; // parser to read and convert samples

  // serial communication parameters
  std::string device_comm_name_; // serial port device file name
  SerialComm * comm_; // serial communication device
  int baud_rate; // serial port baud rate (RS422)
  static const SerialComm::E_DataBits DATA_BITS = SerialComm::DB8;
  static const SerialComm::E_Parity PARITY = SerialComm::NO_PARITY;
  static const SerialComm::E_StopBits STOP_BITS = SerialComm::ONE_STOP_BIT;
  static const short int READ_TIMEOUT_MS = 500;

};

} // namespace

#endif // IMUSAMPLER_H
