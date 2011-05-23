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
#ifndef MEMS_STANDARD_IMU_HEADER_H
#define MEMS_STANDARD_IMU_HEADER_H

#include "IIMUHeader.h"
#include <vector>

namespace mems
{

class StandardIMUHeader : public IIMUHeader
{
public:
    StandardIMUHeader(void);
    ~StandardIMUHeader(void);

    void setHeaderAndChecksum(unsigned char * header, 
        unsigned char checksum);

	std::vector<unsigned char> getBytes();
    unsigned short getMessageSize() const;
    unsigned short getDeviceID() const;
    unsigned short getMessageID() const;
    unsigned char getChecksum() const;
	unsigned short getCounter() const;
    unsigned short getSize();

    static const unsigned short HEADER_NO_CHECKSUM_SIZE = 13;
    static const unsigned short HEADER_PLUS_CHECKSUM_SIZE = 14;
    static const unsigned short NUM_SYNCH_BYTES = 4;
    static const unsigned char  SYNCH_BYTE;

private: 
    static const unsigned short IDX_MESSAGE_SIZE;
    static const unsigned short IDX_DEVICE_ID;
    static const unsigned short IDX_MESSAGE_ID;
    static const unsigned short IDX_COUNTER;
	
	 std::vector<unsigned char>	m_header;
    unsigned char				m_checksum;
};

} // mems

#endif // MEMS_STANDARD_IMU_HEADER_H