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
#include "IMUTempPayload.h"

using namespace mems;

const unsigned short IMUTempPayload::BYTES_PER_SENSOR = 2;


IMUTempPayload::IMUTempPayload(int numberOfSensors)
: m_numTempSensors(numberOfSensors), 
  m_payloadSizeBytes(BYTES_PER_SENSOR * numberOfSensors),
  m_payloadSizeCounts(m_payloadSizeBytes / BYTES_PER_SENSOR), 
  m_payload(m_payloadSizeBytes)
{
    IIMUPayload::setTransform(&m_defaultSDT);
}

IMUTempPayload::IMUTempPayload
(
    ISensorDataTransform * transform,
    int numberOfSensors
) : m_numTempSensors(numberOfSensors), 
    m_payloadSizeBytes(BYTES_PER_SENSOR * numberOfSensors),
    m_payloadSizeCounts(m_payloadSizeBytes / BYTES_PER_SENSOR), 
    m_payload(m_payloadSizeBytes)
{
    IIMUPayload::setTransform(transform);
}

IMUTempPayload::~IMUTempPayload(void)
{
}

void IMUTempPayload::setPayload(unsigned char * payload)
{
	for(int i = 0; i < m_payloadSizeBytes; ++i)
		m_payload[i] = payload[i];
}

std::vector<unsigned char> IMUTempPayload::getBytes()
{
	return(m_payload);
}

std::vector<short> IMUTempPayload::getCounts()
{
	std::vector<short> data(m_payloadSizeCounts);

	for(int i = 0; i < m_payloadSizeCounts; ++i)
	{
		data[i] = getTransform()->sample2Counts(
			&m_payload[i * BYTES_PER_SENSOR]);
	}

	return(data);
}

std::vector<double> IMUTempPayload::getEngUnits()
{
	std::vector<short> counts = getCounts();
	std::vector<double> data(counts.size());
	std::vector<short>::iterator iter;
	int i = 0;

	for(iter = counts.begin(); iter != counts.end(); ++iter, ++i)
	{
      data[i] = getTransform()->counts2Celsius(*iter);
	}

	return(data);
}

std::vector<double> IMUTempPayload::getVolts()
{
	std::vector<short> counts = getCounts();
	std::vector<double> data(counts.size());
	std::vector<short>::iterator iter = counts.begin();
	
	for(int i = 0; iter != counts.end(); ++iter, ++i)
	{
      data[i] = getTransform()->counts2TempVolts( *iter );
	}

	return(data);
}

unsigned short IMUTempPayload::getSize()
{
    return(static_cast<unsigned short>(m_payload.size()));
}
