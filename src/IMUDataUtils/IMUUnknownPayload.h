/*****************************************************************************
 * IMUUnknownPayload
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Handles a Unknown payload.  Will simply look at the number of bytes
 *          given in the header, subtract off the header and checksum,
 *          and assemble what's left into 2-byte payload values.  Has no
 *          concept of data in 'units', so if units are specified will still
 *          print the results in volts.  
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
 *   9/1/06: Initial Release
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_IMU_UNKNOWN_PAYLOAD
#define MEMS_IMU_UNKNOWN_PAYLOAD

#include "IIMUPayload.h"
#include "SDTDefault.h"

namespace mems
{

// forward declaration
class ISensorDataTransform;

class IMUUnknownPayload : public IIMUPayload
{
public:
  IMUUnknownPayload(int payloadSizeBytes);
  IMUUnknownPayload(int payloadSizeBytes, ISensorDataTransform * transform);
  virtual ~IMUUnknownPayload(void);

  void setPayload(unsigned char * payload);
  std::vector<unsigned char> getBytes();
  std::vector<short> getCounts();
  std::vector<double> getEngUnits();
  std::vector<double> getVolts();
  unsigned short getSize();

private:
  static const short BYTES_PER_ELEMENT;

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
  double m_gyroRange;
  double m_accelRange;
  double m_magRange;
};

} // mems

#endif // MEMS_IMU_UNKNOWN_PAYLOAD
