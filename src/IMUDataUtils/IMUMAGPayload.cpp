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
#include "IMUMAGPayload.h"

using namespace mems;

const unsigned short IMUMAGPayload::NUM_SENSORS = 3;
const unsigned short IMUMAGPayload::AXES_PER_SENSOR = 3;
const unsigned short IMUMAGPayload::BYTES_PER_AXIS = 2;
const unsigned short IMUMAGPayload::PAYLOAD_SIZE_BYTES = NUM_SENSORS * AXES_PER_SENSOR * BYTES_PER_AXIS;
const unsigned short IMUMAGPayload::PAYLOAD_SIZE_COUNTS = PAYLOAD_SIZE_BYTES / BYTES_PER_AXIS;

IMUMAGPayload::IMUMAGPayload(double defaultGyroRange, double defaultAccelRange, double defaultMagRange,
                             int numGyroSensors, int numAccelSensors, int numMagSensors,
                             ISensorDataTransform* transform) :
  m_payload(PAYLOAD_SIZE_BYTES)
{
  setGyroRange(defaultGyroRange);
  setAccelRange(defaultAccelRange);
  setMagRange(defaultMagRange);

  IIMUPayload::setTransform(&m_defaultSDT);
}

IMUMAGPayload::~IMUMAGPayload(void)
{
}

void IMUMAGPayload::setGyroRange(double range)
{
  setGyroRange(range, range, range);
}

void IMUMAGPayload::setAccelRange(double range)
{
  setAccelRange(range, range, range);
}

void IMUMAGPayload::setMagRange(double range)
{
  setMagRange(range, range, range);
}

void IMUMAGPayload::setGyroRange(double xRange, double yRange, double zRange)
{
  m_gyroRange[0] = xRange;
  m_gyroRange[1] = yRange;
  m_gyroRange[2] = zRange;
}

void IMUMAGPayload::setAccelRange(double xRange, double yRange, double zRange)
{
  m_accelRange[0] = xRange;
  m_accelRange[1] = yRange;
  m_accelRange[2] = zRange;
}

void IMUMAGPayload::setMagRange(double xRange, double yRange, double zRange)
{
  m_magRange[0] = xRange;
  m_magRange[1] = yRange;
  m_magRange[2] = zRange;
}

double IMUMAGPayload::getGyroRange()
{
  return (m_gyroRange[0]);
}

double IMUMAGPayload::getAccelRange()
{
  return (m_accelRange[0]);
}

double IMUMAGPayload::getMagRange()
{
  return (m_magRange[0]);
}

void IMUMAGPayload::getGyroRange(double rangeArray[])
{
  rangeArray[0] = m_gyroRange[0];
  rangeArray[1] = m_gyroRange[1];
  rangeArray[2] = m_gyroRange[2];
}

void IMUMAGPayload::getAccelRange(double rangeArray[])
{
  rangeArray[0] = m_accelRange[0];
  rangeArray[1] = m_accelRange[1];
  rangeArray[2] = m_accelRange[2];
}

void IMUMAGPayload::getMagRange(double rangeArray[])
{
  rangeArray[0] = m_magRange[0];
  rangeArray[1] = m_magRange[1];
  rangeArray[2] = m_magRange[2];
}

void IMUMAGPayload::setPayload(unsigned char * payload)
{
  for (int i = 0; i < PAYLOAD_SIZE_BYTES; ++i)
    m_payload[i] = payload[i];
}

std::vector<unsigned char> IMUMAGPayload::getBytes()
{
  return (m_payload);
}

std::vector<short> IMUMAGPayload::getCounts()
{
  std::vector<short> data(PAYLOAD_SIZE_COUNTS);

  for (int i = 0; i < PAYLOAD_SIZE_COUNTS; ++i)
  {
    data[i] = getTransform()->sample2Counts(&m_payload[i * BYTES_PER_AXIS]);
  }

  return (data);
}

std::vector<double> IMUMAGPayload::getEngUnits()
{
  std::vector<short> counts = getCounts();
  std::vector<double> data(counts.size());
  std::vector<short>::iterator iter;
  int i = 0;
  double range;

  for (iter = counts.begin(); iter != counts.end(); ++iter, ++i)
  {
    if (i < ACCEL)
      range = m_gyroRange[i];
    else if (i < MAG)
      range = m_accelRange[i - 3];
    else
      range = m_magRange[i - 6];

    data[i] = getTransform()->counts2EngUnits(*iter, range);
  }

  return (data);
}

std::vector<double> IMUMAGPayload::getVolts()
{
  std::vector<short> counts = getCounts();
  std::vector<double> data(counts.size());
  std::vector<short>::iterator iter = counts.begin();

  for (int i = 0; iter != counts.end(); ++iter, ++i)
  {
    data[i] = getTransform()->counts2Volts(*iter);
  }

  return (data);
}

unsigned short IMUMAGPayload::getSize()
{
  return (static_cast<unsigned short> (m_payload.size()));
}

