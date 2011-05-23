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

#include "IMUHiGPayload.h"
#include "SensorDataUtils.h"

using namespace mems;

const unsigned short IMUHiGPayload::AXES_PER_SENSOR = 1;
const unsigned short IMUHiGPayload::BYTES_PER_AXIS = 2;


IMUHiGPayload::IMUHiGPayload(double range, unsigned short numSensors)
   : m_range(range), m_numSensors(numSensors)
{
    IIMUPayload::setTransform(&m_defaultSDT);
    m_payloadSizeBytes = m_numSensors * AXES_PER_SENSOR * BYTES_PER_AXIS;
    m_payloadSizeCounts = m_payloadSizeBytes / BYTES_PER_AXIS;    
    m_payload.resize( m_payloadSizeBytes );
}

IMUHiGPayload::IMUHiGPayload(double range, ISensorDataTransform * transform,
                             unsigned short numSensors)
   : IIMUPayload(transform), m_range(range), m_numSensors(numSensors)
{
   m_payloadSizeBytes = m_numSensors * AXES_PER_SENSOR * BYTES_PER_AXIS;
   m_payloadSizeCounts = m_payloadSizeBytes / BYTES_PER_AXIS;
   m_payload.resize( m_payloadSizeBytes );
}

IMUHiGPayload::~IMUHiGPayload(void)
{
}

void IMUHiGPayload::setRange(double range)
{
	m_range = range;
}

double IMUHiGPayload::getRange()
{
    return(m_range);
}

void IMUHiGPayload::setPayload(unsigned char * payload)
{
	for(int i = 0; i < m_payloadSizeBytes; ++i)
		m_payload[i] = payload[i];
}

std::vector<unsigned char> IMUHiGPayload::getBytes()
{
	return(m_payload);
}

std::vector<short> IMUHiGPayload::getCounts()
{
	std::vector<short> data(m_payloadSizeCounts);

	for(int i = 0; i < m_payloadSizeCounts; ++i)
	{
		data[i] = getTransform()->sample2Counts(
			&m_payload[i * BYTES_PER_AXIS]);
	}

	return(data);
}

std::vector<double> IMUHiGPayload::getEngUnits()
{
	std::vector<short> counts = getCounts();
	std::vector<double> data(counts.size());
	std::vector<short>::iterator iter;
	int i = 0;
   
	for(iter = counts.begin(); iter != counts.end(); ++iter, ++i)
	{
		data[i] = getTransform()->counts2EngUnits(*iter, m_range);
	}

	return(data);
}

std::vector<double> IMUHiGPayload::getVolts()
{
	std::vector<short> counts = getCounts();
	std::vector<double> data(counts.size());
	std::vector<short>::iterator iter = counts.begin();
	
	for(int i = 0; iter != counts.end(); ++iter, ++i)
	{
		data[i] = getTransform()->counts2Volts( *iter );
	}

	return(data);
}

unsigned short IMUHiGPayload::getSize()
{
    return(static_cast<unsigned short>(m_payload.size()));
}

void IMUHiGPayload::setNumSensors( unsigned short numSensors )
{
   m_numSensors = numSensors;
   m_payloadSizeBytes = m_numSensors * AXES_PER_SENSOR * BYTES_PER_AXIS;
   m_payloadSizeCounts = m_payloadSizeBytes / BYTES_PER_AXIS;    
   m_payload.resize( m_payloadSizeBytes );
}

