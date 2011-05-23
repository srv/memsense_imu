/**
 * @file IMUSampler.cpp
 * @brief Sampler for the Memsense family of IMUs, implementation
 * @author Joan Pau Beltran
 * @date 2011-02-08
 *
 * This file implements the IMU sampler class for the family of such devices
 * provided by Memsense. It uses the libraries provided by Memsense, with
 * some modifications to run on GNU/Linux (and probably other UNIX-alike
 * systems).
 */

#include "IMUSampler.h"
#include "IMUDataUtils/SDTDefault.h"     // data transform (conversion) to engineering uints
#include "IMUDataUtils/SDTDefaultPre.h"  // data transform (conversion) to volts
#include "IMUDataUtils/CommonUtils.h"    // common utilities such as string conversions of device type names

/** Constructor
 * @return
 */
mems::IMUSampler::IMUSampler()
: device_type_(DT_INVALID), gyro_range_(0.0), accel_range_(0.0),
  mag_range_(0.0), hig_range_(0.0),
  data_format_(DF_INVALID), parser_(0), device_comm_name_(""), comm_(0)
{
}

/** Initialize the parser to process the data read from the IMU
 * An internal parser is created by this function. It should be called before
 * any read trial. Device type should be known, as well as measurement ranges.
 * Number of sensors is inferred from the device type.
 * Magnitude ranges not common to all IMUs (temperatures, hi-g) are optional.
 * By default the internal transform is set to output real values in engineering
 * units.
 * @param device_type Memsense IMU type identifier
 * @param gyro_range gyroscope measurement range (degrees/s)
 * @param accel_range accelerometer measurement range (g/s)
 * @param mag_range magnetometer measurement range (gauss)
 * @param hig_range hi-g measurement range (?)
 */
void mems::IMUSampler::initParser(E_DeviceType device_type, double gyro_range,
                                  double accel_range, double mag_range, double hig_range)

{
  // Delete previous parser if any
  deleteParser();
  
  // Update device parameters
  device_type_ = device_type;
  device_type_name_ = deviceType2Name(device_type);
  gyro_range_ = gyro_range;
  accel_range_ = accel_range;
  mag_range_ = mag_range;
  hig_range_ = hig_range;
  num_hig_axes_ = getNumberOfHiGAxes(device_type);
  num_temp_sensors_ = getNumberOfTempSensors(device_type);

  // Set sample parser
  if (num_hig_axes_ > 0)
  {
    if (hig_range_==0.0)
      throw IMUError("Wrong hi-g range for device type " + device_type_name_);
    parser_ = STDIMUSample::getStandardSample(device_type, gyro_range,
                                              accel_range, mag_range, hig_range);
  }
  else
  {
    parser_ = STDIMUSample::getStandardSample(device_type, gyro_range,
                                              accel_range, mag_range);
  }
  if (!parser_)
    throw IMUError("Error initializing parser for device type " + device_type_name_);
}

/** Clean up parser's internal transform and delete current parser
 *
 */
void mems::IMUSampler::deleteParser()
{
  if( parser_ )
  {
    if ( parser_->getTransform() )
      delete parser_->getTransform();
    delete parser_;
  }
}

/** Set the desired output format
 * Sets the parser's internal transform according to the desired format
 * @param format output data in ENGINEERING UNITS (deg/sec, g, gauss) or VOLTS.
 *               Only affects getDataReal().
 */
void mems::IMUSampler::setFormat(E_DataFormatType format)
{
  if (!parser_)
    throw IMUError("Parser not initialized");
  if (parser_->getTransform())
    delete parser_->getTransform();
  data_format_ = format;
  switch (format)
  {
    case ENGINEERING_UNITS :
      parser_->setTransform(new SDTDefault());
      break;
    case VOLTS :
      parser_->setTransform(new SDTDefaultPre());
      break;
    default:
      throw IMUError("Invalid IMU data format " + dataFormatType2Name(format) );
  }
}

/** Get the current format
 * @return the current format.
 */
mems::E_DataFormatType mems::IMUSampler::getFormat() const
{
  return data_format_;
}

/** Get the current device name as string
 * @return the device type name as string.
 */
std::string mems::IMUSampler::getDeviceName() const
{
  return device_type_name_;
}

/** Open the serial port device file for communication
 * Opens the serial device and configure it following Memsense specifications.
 * On errors an exception is thrown.
 */
void mems::IMUSampler::openComm(std::string device_name, unsigned long baud_rate)
{
  int status;

  if (comm_)
  {
    if ( ! (comm_->closeDevice(status)) )
      throw IMUError("Error closing previously opened serial device " + device_comm_name_ );
    delete comm_;
    comm_ = 0;
  }

  comm_ = new SerialComm();
  SerialComm::SDeviceData devData;
  devData.m_comPort = device_name;

  if (! (comm_->openDevice(&devData, status)) )
  {
    delete comm_;
    comm_ = 0;
    throw IMUError("Error opening serial device " + device_name );
  }

  if (! ( comm_->setBaudRate(baud_rate) &&
          comm_->setDataBits(DATA_BITS) &&
          comm_->setParity(PARITY) &&
          comm_->setStopBits(STOP_BITS) &&
          comm_->setReadTimeout(READ_TIMEOUT_MS) ) )
  {
    delete comm_;
    comm_ = 0;
    throw IMUError("Error configuring serial device " + device_name );
  }
}

/** Closes the serial communication device (if it is already open)
 * On errors an exception is thrown.
 */
void mems::IMUSampler::closeComm()
{
  if( comm_ )
  {
    int status;
    if ( !(comm_->closeDevice(status)) )
      throw IMUError("Error closing serial device " + device_comm_name_);
    delete comm_;
    comm_ = 0;
  }
}

/** Get the name of the serial port device file
 * @return serial port device name (including full path)
 */
std::string mems::IMUSampler::getCommName() const
{
  return device_comm_name_;
}

/** Destructor
 * Delete the internal parser (and its internal transform) if needed,
 * and close serial port if it is open.
 * @return
 */
mems::IMUSampler::~IMUSampler()
{
  if ( parser_ )
  {
    deleteParser();
  }
  if ( comm_ )
  {
    closeComm();
  }
}

/** Poll the device to receive a sample
 * When the checksum is not valid an exception is thrown. 
 * Since reading failures are quite common due to
 * lack of synchronization, not finding a sample in the bytes read from the 
 * serial port does not throw an exception, but the return value is false.
 * 
 * @return whether a sample was found in the bytes read from the serial port
 *
 * @todo Check if buffer should be flushed before reading the sample.
 */
bool mems::IMUSampler::readSample()
{
  int status;
//  if(!comm_->flushBuffer(status))
//    throw IMUError("Error flushing serial device" + device_comm_name_);

  // use the parser to read up the sample
  if( ! parser_->readSample(comm_, status) )
    return false;
//    throw IMUError("Error reading " + device_type_name_ +
//                   " sample from serial device " + device_comm_name_);

  // verify the checksum and number of bytes read is correct.
  if(!parser_->isValidChecksum())
    throw IMUError("Error invalid checksum for " + device_type_name_ + " sample ");
  return true;
}

//Forward declaration of auxiliar functions for getData functions
template<typename T>
void fillMAGData(const std::vector<T>& MAG_data,
                 std::vector<T>* gyro, std::vector<T>* accel, std::vector<T>* mag);

template<typename T>
void fillHigData(const std::vector<T>& hig_data, std::vector<T>* hig);

template<typename T>
void fillTempData(const std::vector<T>& temp_data, std::vector<T>* temp);

/** Parse last read sample's values as floats (engineering units or volts)
 * Parse the last read sample to extract received values as engineering
 * units or volts depending on the format set by initParser() or setFormat().
 * If some argument is null the values of the respective magnitude in the
 * sample (if any) are ignored.
 * On errors (e.g. bad or no data received from the IMU) it throws an exception
 * @param gyro  gyroscope measurements (deg/s or volts)
 * @param accel accelerometer measurements (g or volts)
 * @param mag magnetometer measurements(gauss or volts)
 * @param temp temperature measurements (ºC or volts)
 * @param hig hi-g accelerometer measurements (? or volts)
 */
void mems::IMUSampler::getDataReal(std::vector<double>* gyro,
                                   std::vector<double>* accel,
                                   std::vector<double>* mag,
                                   std::vector<double>* temp,
                                   std::vector<double>* hig)
{
  // get payload (ignore header data, that contains counter data among others)
  // parser_->getHeader()->getBytes();
  // parser_->getHeader()->getCounter();
  std::vector<IIMUPayload *> payloads = parser_->getPayload();

  // Extract data from payload, which is of the form
  //    {gyr, acc, mag} [{hig}] [{temp}]
  // where {} denotes a specific payload and [] means optional
  if (data_format_==VOLTS)
  {
    int i=0;
    fillMAGData(payloads[i++]->getVolts(), gyro, accel, mag );
    if (num_hig_axes_>0)
      fillHigData(payloads[i++]->getVolts(), hig);
    if (num_temp_sensors_>0)
      fillTempData(payloads[i++]->getVolts(), temp);
  }
  else
  {
    int i=0;
    fillMAGData(payloads[i++]->getEngUnits(), gyro, accel, mag );
    if (num_hig_axes_>0)
      fillHigData(payloads[i++]->getEngUnits(), hig);
    if (num_temp_sensors_>0)
      fillTempData(payloads[i++]->getEngUnits(), temp);
  }
}

/** Parse last read sample's values as counts (short integers)
 * Parse the last read sample to extract received values as 16 bit integers
 * If some argument is null the values of the respective magnitude in the
 * sample (if any) are ignored.
 * On errors (e.g. bad or no data received from the IMU) it throws an exception
 * @param gyro  gyroscope measurements (DC bits)
 * @param accel accelerometer measurements (DC bits)
 * @param mag magnetometer measurements(DC bits)
 * @param temp temperature measurements (DC bits)
 * @param hig hi-g accelerometer measurements (DC bits)
 */
void mems::IMUSampler::getDataCounts(std::vector<short>* gyro,
                                     std::vector<short>* accel,
                                     std::vector<short>* mag,
                                     std::vector<short>* temp,
                                     std::vector<short>* hig)
{
  std::vector<IIMUPayload *> payloads = parser_->getPayload();
  int i=0;
  fillMAGData(payloads[i++]->getCounts(), gyro, accel, mag );
  if (num_hig_axes_>0)
    fillHigData(payloads[i++]->getCounts(), hig);
  if (num_temp_sensors_>0)
    fillTempData(payloads[i++]->getCounts(), temp);
}


/** Parse last read sample's values as a raw burst of bytes.
 * Parse the last read sample to extract received values as bytes.
 * If some argument is null the values of the respective magnitude in the
 * sample (if present) are ignored.
 * On errors (e.g. bad or no data received from the IMU) it throws an exception
 * @param gyro  gyroscope measurements (bytes)
 * @param accel accelerometer measurements (bytes)
 * @param mag magnetometer measurements(bytes)
 * @param temp temperature measurements (bytes)
 * @param hig hi-g accelerometer measurements (bytes)
 */
 void mems::IMUSampler::getDataBytes(std::vector<unsigned char>* gyro,
                                    std::vector<unsigned char>* accel,
                                    std::vector<unsigned char>* mag,
                                    std::vector<unsigned char>* temp,
                                    std::vector<unsigned char>* hig)
{
  std::vector<IIMUPayload *> payloads = parser_->getPayload();
  int i=0;
  fillMAGData(payloads[i++]->getBytes(), gyro, accel, mag );
  if (num_hig_axes_>0)
    fillHigData(payloads[i++]->getBytes(), hig);
  if (num_temp_sensors_>0)
    fillTempData(payloads[i++]->getBytes(), temp);
}

////////////////////////////////////////////////////////////////////////////////
// Auxiliar functions
////////////////////////////////////////////////////////////////////////////////

template<typename T>
void fillMAGData(const std::vector<T>& MAG_data,
                 std::vector<T>* gyro, std::vector<T>* accel, std::vector<T>* mag)
{
  if (gyro)
  {
    gyro->clear();
    for (int i=0; i<3; i++)
      gyro->push_back(MAG_data[mems::IMUMAGPayload::GYRO + i]);
  }
  if (accel)
  {
    accel->clear();
    for (int i=0; i<3; i++)
      accel->push_back(MAG_data[mems::IMUMAGPayload::ACCEL + i]);
  }
  if (mag)
  {
    mag->clear();
    for (int i=0; i<3; i++)
      mag->push_back(MAG_data[mems::IMUMAGPayload::MAG + i]);
  }
}

template<typename T>
void fillHigData(const std::vector<T>& hig_data, std::vector<T>* hig)
{
  if (hig)
    *hig = hig_data;
}

template<typename T>
void fillTempData(const std::vector<T>& temp_data, std::vector<T>* temp)
{
  if (temp)
    *temp = temp_data;
}



