/*****************************************************************************
 * IMUTempPayload
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Handles the temperature payload.  The standard temperature
 *          payload is comprised of three temperatures - one for each gyro.
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
#ifndef MEMS_IMUTEMP_PAYLOAD_H
#define MEMS_IMUTEMP_PAYLOAD_H

#include "IIMUPayload.h"
#include "SDTDefault.h"

namespace mems
{

class IMUTempPayload : public IIMUPayload
{
public:
  IMUTempPayload(int numberOfSensors = 3);
  IMUTempPayload(ISensorDataTransform * transform, int numberOfSensors = 3);
  virtual ~IMUTempPayload(void);

  void setPayload(unsigned char * payload);
  std::vector<unsigned char> getBytes();
  std::vector<short> getCounts();
  std::vector<double> getEngUnits();
  std::vector<double> getVolts();
  unsigned short getSize();

private:
  static const unsigned short BYTES_PER_SENSOR;
  unsigned short m_numTempSensors;
  unsigned short m_payloadSizeBytes;
  unsigned short m_payloadSizeCounts;

  SDTDefault m_defaultSDT;
  std::vector<unsigned char> m_payload;
};

} // mems

#endif // MEMS_IMUTEMP_PAYLOAD_H
