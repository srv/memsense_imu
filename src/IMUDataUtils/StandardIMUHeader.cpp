/*****************************************************************************
 * StandardIMUHeader
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Utility class for processing the standard 13-byte IMU header.  This
 *      header is currently present on all MEMSense IMU's.
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
#include "StandardIMUHeader.h"
#include "SensorDataUtils.h"

using namespace mems;

const unsigned short StandardIMUHeader::IDX_MESSAGE_SIZE = 4;
const unsigned short StandardIMUHeader::IDX_DEVICE_ID = 5;
const unsigned short StandardIMUHeader::IDX_MESSAGE_ID = 6;
const unsigned short StandardIMUHeader::IDX_COUNTER = 7;
const unsigned char  StandardIMUHeader::SYNCH_BYTE = 0xFF;

StandardIMUHeader::StandardIMUHeader(void) : m_header(HEADER_NO_CHECKSUM_SIZE)
{

}

StandardIMUHeader::~StandardIMUHeader(void)
{
}

void StandardIMUHeader::setHeaderAndChecksum
(
    unsigned char * header, 
    unsigned char checksum
)
{
	m_header.clear();
    for(int i = 0; i < HEADER_NO_CHECKSUM_SIZE; ++i)
		m_header.push_back(header[i]);
    
    m_checksum = checksum;
}

std::vector<unsigned char> StandardIMUHeader::getBytes()
{
	return(m_header);
}

unsigned short StandardIMUHeader::getMessageSize() const
{
    return(m_header[IDX_MESSAGE_SIZE]);
}

unsigned short StandardIMUHeader::getDeviceID() const
{
    return(m_header[IDX_DEVICE_ID]);
}

unsigned short StandardIMUHeader::getMessageID() const
{
    return(m_header[IDX_MESSAGE_ID]);
}

unsigned char StandardIMUHeader::getChecksum() const
{
    return(m_checksum);
}

unsigned short StandardIMUHeader::getCounter() const
{
	return(SensorDataUtils::sample2Counts(
		m_header[IDX_COUNTER], m_header[IDX_COUNTER + 1]));
}

unsigned short StandardIMUHeader::getSize()
{
    return(static_cast<unsigned short>(m_header.size()) + sizeof(m_checksum));
}
