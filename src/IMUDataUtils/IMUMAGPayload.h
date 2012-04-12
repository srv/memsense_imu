/*****************************************************************************
 * IMUMAGPayload
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Handles the gyro, accelerometer and magnetometer payload.  This 
 *          payload is currently standard on all MEMSense IMU's.  The 
 *          payload is expected to be order as Gyro/Accel/Mag.
 * 
 *
 * Requirements:
 * Compiling: 
 *      
 * Linking:
 *      
 * Execution:
 *
 *
 * Modifications:
 *
 *   3/13/06: Initial Release
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_IMU_MAG_PAYLOAD_H
#define MEMS_IMU_MAG_PAYLOAD_H

#include "IIMUPayload.h"
#include "SDTDefault.h"
#include <cstddef> // NULL macro

namespace mems
{

// forward declaration
class ISensorDataTransform;

class IMUMAGPayload : public IIMUPayload
{
public:
  IMUMAGPayload(double defaultGyroRange, double defaultAccelRange, double defaultMagRange, int numGyroSensors = 3,
                int numAccelSensors = 3, int numMagSensors = 3, ISensorDataTransform* transform = NULL);

  virtual ~IMUMAGPayload(void);

  void setGyroRange(double range);
  void setAccelRange(double range);
  void setMagRange(double range);

  void setGyroRange(double xRange, double yRange, double zRange);
  void setAccelRange(double xRange, double yRange, double zRange);
  void setMagRange(double xRange, double yRange, double zRange);

  double getGyroRange();
  double getAccelRange();
  double getMagRange();

  void getGyroRange(double rangeArray[]);
  void getAccelRange(double rangeArray[]);
  void getMagRange(double rangeArray[]);

  void setPayload(unsigned char * payload);
  std::vector<unsigned char> getBytes();
  std::vector<short> getCounts();
  std::vector<double> getEngUnits();
  std::vector<double> getVolts();
  unsigned short getSize();

private:
  static const unsigned short NUM_SENSORS;
  static const unsigned short AXES_PER_SENSOR;
  static const unsigned short BYTES_PER_AXIS;
  static const unsigned short PAYLOAD_SIZE_BYTES;
  static const unsigned short PAYLOAD_SIZE_COUNTS;

  /**
   * Index of beginning of sensor groups within the *counts* payload.
   * For example, the Accel X value starts at index 3.
   **/
  enum E_SENSOR_INDEX
  {
    GYRO = 0, ACCEL = 3, MAG = 6
  };

  SDTDefault m_defaultSDT;
  std::vector<unsigned char> m_payload;
  double m_range;
  double m_gyroRange[3];
  double m_accelRange[3];
  double m_magRange[3];
};

} // mems

#endif // MEMS_IMU_MAG_PAYLOAD_H
