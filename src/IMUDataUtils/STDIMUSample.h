#ifndef __MEMS_STANDARD_IMU_SAMPLE_H__
#define __MEMS_STANDARD_IMU_SAMPLE_H__

#include "IIMUHeader.h"
#include "StandardIMUHeader.h"
#include "IMUMAGPayload.h"
#include "IMUHiGPayload.h"
#include "IMUTempPayload.h"
#include "Types.h"

#include <vector>
#include <cstddef> // NULL macro

namespace mems
{
/**
 * @class STDIMUSample
 * This is the class that is instantiated by all of the more-specific sample
 * classes. It is general enough to handle all current devices except the ICG.
 **/
class STDIMUSample
{
public:
  void setGyroRange(double range);
  void setAccelRange(double range);
  void setMagRange(double range);
  void setHiGAccelRange(double range);

  void setNumberGyroSensors(unsigned short sensors);
  void setNumberAccelSensors(unsigned short sensors);
  void setNumberMagSensors(unsigned short sensors);
  void setNumberHiGSensors(unsigned short sensors);
  void setNumberTemperatureSensors(unsigned short sensors);

  bool readSample(IComm * comm, int & status, const unsigned short findSynchMaxConsecutiveFailures = 10);
  void setSample(unsigned char * sample);

  std::vector<unsigned char> getBytes();
  IIMUHeader * getHeader();
  std::vector<IIMUPayload *> getPayload();

  unsigned short getHeaderSize();
  unsigned short getPayloadSize();
  unsigned short getSize();

  bool isValidChecksum();

  STDIMUSample* clone();

  void setTransform(ISensorDataTransform* transform);
  ISensorDataTransform* getTransform();

  STDIMUSample(unsigned short numGyroSensors, double gyroRange, unsigned short numAccelSensors, double accelRange,
               unsigned short numMagSensors, double magRange, unsigned short numHiGSensors, double hiGRange,
               unsigned short numTempSensors, ISensorDataTransform* transform = NULL);

  //This function takes care of allocating a STDIMUSample for you, based
  //upon the information passed in. It returns NULL in case of error, which
  //usually means that device type needs more sensor range values than you
  //provided, or a device type was requested that the STDIMUSample cannot
  //emulate. The gyro, accel, and mag ranges are required because every
  //device to which we can connect has at least those three sensors.
  static STDIMUSample* getStandardSample(mems::E_DeviceType deviceType, double gyroRange, double accelRange,
                                         double magRange, double hiGRange = SENSOR_UNAVAILABLE_SENSOR_RANGE);

  //This function also allocates a STDIMUSample, but takes different
  //information that can be read from the header.
  static STDIMUSample* getStandardSample(unsigned short deviceCode, int packetSize, double gyroRange = 300,
                                         double accelRange = 2, double hiGRange = 10);

private:

  void updateIndicesAndPayloads();
  short m_idxGyro;
  short m_idxAccel;
  short m_idxMag;
  short m_idxHiG;
  short m_idxTemp;
  short m_idxChecksum;

  short m_numGyroSensors;
  short m_numAccelSensors;
  short m_numMagSensors;
  short m_numHiGSensors;
  short m_numTempSensors;
  short m_sampleSize;

  double m_gyroRange;
  double m_accelRange;
  double m_magRange;
  double m_hiGRange;

  unsigned char* m_bytes;
  unsigned char* m_commBuffer;

  mems::StandardIMUHeader m_header;
  mems::IMUMAGPayload* m_magPayload;
  mems::IMUHiGPayload* m_higPayload;
  mems::IMUTempPayload* m_tempPayload;
  mems::ISensorDataTransform* m_transform;

  static const unsigned short SENSOR_SIZE_IN_BYTES;
  static const int SENSOR_IDX_UNKNOWN;
  static const double SENSOR_UNAVAILABLE_SENSOR_RANGE;
  static const int USE_DEFAULT_TEMP_SENSORS;

};
}

#endif
