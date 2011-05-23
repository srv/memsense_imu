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
 *          print the results in EngUnits.  
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
#include "IMUUnknownPayload.h"

using namespace mems;

const short IMUUnknownPayload::BYTES_PER_ELEMENT = 2;

IMUUnknownPayload::IMUUnknownPayload(int payloadSizeBytes) : m_payload(payloadSizeBytes, 0)
{
	IIMUPayload::setTransform(&m_defaultSDT);
}

IMUUnknownPayload::IMUUnknownPayload
(
    int payloadSizeBytes, 
    ISensorDataTransform * transform
) : m_payload(payloadSizeBytes, 0)
{
    IIMUPayload::setTransform(transform);	
}

IMUUnknownPayload::~IMUUnknownPayload(void)
{
}

void IMUUnknownPayload::setPayload(unsigned char * payload)
{
    for(std::vector<unsigned char>::size_type i = 0; i < m_payload.size(); ++i)
		m_payload[i] = payload[i];
}

std::vector<unsigned char> IMUUnknownPayload::getBytes()
{
	return(m_payload);
}

std::vector<short> IMUUnknownPayload::getCounts()
{
	std::vector<short> data(m_payload.size()/BYTES_PER_ELEMENT);

	for(std::vector<unsigned char>::size_type i = 0; i < m_payload.size()/BYTES_PER_ELEMENT; ++i)
	{
		data[i] = getTransform()->sample2Counts(
			&m_payload[i * BYTES_PER_ELEMENT]);
	}

	return(data);
}


std::vector<double> IMUUnknownPayload::getEngUnits()
{
	std::vector<short> counts = getCounts();
	std::vector<double> data(counts.size());
	std::vector<short>::iterator iter;
	int i = 0;

	for(iter = counts.begin(); iter != counts.end(); ++iter, ++i)
		data[i] = getTransform()->counts2EngUnits(*iter, 1.0);

	return(data);
}

std::vector<double> IMUUnknownPayload::getVolts()
{
   std::vector<short> counts = getCounts();
   std::vector<double> data( counts.size() );
   
   std::vector<short>::iterator shortIter = counts.begin();
   
   for(int i = 0; shortIter != counts.end(); ++shortIter, ++i)
      data[i] = getTransform()->counts2Volts( *shortIter );

   return data;

}

unsigned short IMUUnknownPayload::getSize()
{
    return(static_cast<unsigned short>(m_payload.size()));
}


