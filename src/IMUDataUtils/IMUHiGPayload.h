/*****************************************************************************
 * IMUHiGPayload
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Handles the hi-g accelerometer payload.
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
#ifndef MEMS_IMU_HIG_PAYLOAD_H
#define MEMS_IMU_HIG_PAYLOAD_H

#include "IIMUPayload.h"
#include "SDTDefault.h"

namespace mems
{

class IMUHiGPayload : public IIMUPayload
{
public:
  IMUHiGPayload(double range, unsigned short numSensors = 3);
  IMUHiGPayload(double range, ISensorDataTransform * transform, unsigned short numSensors = 3);
  ~IMUHiGPayload(void);

  void setRange(double range);
  double getRange();
  void setPayload(unsigned char * payload);
  void setNumSensors(unsigned short numSensors);
  std::vector<unsigned char> getBytes();
  std::vector<short> getCounts();
  std::vector<double> getEngUnits();
  std::vector<double> getVolts();
  unsigned short getSize();

private:
  unsigned short m_numSensors;
  static const unsigned short AXES_PER_SENSOR;
  static const unsigned short BYTES_PER_AXIS;
  unsigned short m_payloadSizeBytes;
  unsigned short m_payloadSizeCounts;

  /**
   * Index of beginning of sensor axis within the *counts* payload.
   **/
  enum E_SENSOR_INDEX
  {
    X = 0, Y = 1, Z = 2
  };

  SDTDefault m_defaultSDT;
  std::vector<unsigned char> m_payload;
  double m_range;
};

} // mems

#endif // MEMS_IMU_HIG_PAYLOAD_H
