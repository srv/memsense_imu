/*****************************************************************************
 * UnknownIMUSample
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Handles the reading and parsing of data from an 'unknown' nIMU .
 *      This class is derived from IIMUSample which exposes all necessary
 *      methods for basic reading and parsing of IMU data.  Note that this
 *      concrete class is specifically written such that it does not 
 *      specifically recognize the type of IMU from which it is sampling.
 *      Rather, it assumes each sensor sample/element is a 2-byte value
 *      that cannot be displayed in units since it is unaware of which
 *      element is gyro, accel or mag.  If units is specified it will 
 *      print engineering units in place.
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
#ifndef MEMS_UNKNOWN_IMU_SAMPLE_H
#define MEMS_UNKNOWN_IMU_SAMPLE_H

#include "IIMUSample.h"         
#include "StandardIMUHeader.h"  // using standard IMU header
#include "IMUUnknownPayload.h"  // contain 'generic' payload

namespace mems
{
class ISensorDataTransform;

class UnknownIMUSample : public IIMUSample
{
public:
    UnknownIMUSample(short sampleSizeBytes);
    ~UnknownIMUSample(void);

	bool readSample(IComm * comm, int & status,
                   const unsigned short findSynchMaxConsecutiveFailures = 10);
    void setSample(unsigned char * sample);
	std::vector<unsigned char> getBytes();
	IIMUHeader * getHeader();
	std::vector<IIMUPayload *> getPayload();
    bool isValidChecksum();
    unsigned short getHeaderSize();
    unsigned short getPayloadSize();
    unsigned short getSize();
    IIMUSample * clone();

    void setTransform(ISensorDataTransform * transform);
    ISensorDataTransform * getTransform();


private:
	unsigned char         * m_bytes;
	unsigned char         * m_commBuffer;
    short                   m_sampleSizeBytes;
	StandardIMUHeader	    m_header;
	IMUUnknownPayload       m_payload;
    ISensorDataTransform  * m_transform;
};

} // mems

#endif // MEMS_UNKNOWN_IMU_SAMPLE_H