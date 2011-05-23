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

#include "UnknownIMUSample.h"
#include "SensorDataUtils.h"    // converting data to counts/engineering units/units
#include "CommonUtils.h"        // findSync()
#include "IComm.h"              // communication interface
#include "ISensorDataTransform.h"

using namespace mems;


UnknownIMUSample::UnknownIMUSample
(
    short sampleSizeBytes
) : 
    m_sampleSizeBytes(sampleSizeBytes), 
    m_payload(sampleSizeBytes - StandardIMUHeader::HEADER_PLUS_CHECKSUM_SIZE),
    m_transform(m_payload.getTransform())
{
    m_bytes = m_commBuffer = new unsigned char[m_sampleSizeBytes];
}

UnknownIMUSample::~UnknownIMUSample(void)
{
    delete [] m_commBuffer;
}

IIMUSample * UnknownIMUSample::clone()
{
    UnknownIMUSample * sample;

    sample = new UnknownIMUSample(m_sampleSizeBytes);

    sample->setSample(m_commBuffer);

    return(sample);
}

bool UnknownIMUSample::readSample(
       IComm * comm,
       int & status,
       const unsigned short findSynchMaxConsecutiveFailures)
{
	unsigned long bytesRead;
    bool success = false;
	
    /**
     * No guarantee is made that the data will begin arriving at the sample
     * boundaries (i.e. at beginning of synch bytes).  Therefore, must find
     * the synch bytes and then do a block read of the remaining data to
     * complete the sample.
	 * NOTE: The buffer returns with NUM_SYNCH_BYTES synch bytes prepended.
     **/
    if(mems::findSynch(comm, m_commBuffer, StandardIMUHeader::NUM_SYNCH_BYTES,
            StandardIMUHeader::SYNCH_BYTE, findSynchMaxConsecutiveFailures))
    {
        /** 
        * Read the rest of the sample.  Note the fact that the synch bytes
        * have already been read is accounted for by subtracting them from
        * the overall sample size, and the buffer pointer is incremented,
        * past the synch bytes.
        **/
        success = comm->readData(m_sampleSizeBytes - StandardIMUHeader::NUM_SYNCH_BYTES, 
            m_commBuffer + StandardIMUHeader::NUM_SYNCH_BYTES, bytesRead, status);
    }

    if(success)
	{
        bytesRead = m_sampleSizeBytes;
		setSample(m_commBuffer);
	}

	return(success);
}

void UnknownIMUSample::setSample(unsigned char * sample)
{
	m_bytes = sample;
	m_header.setHeaderAndChecksum(sample, sample[m_sampleSizeBytes - 1]);

    /**
     * NOTE: the header returns size of header + checksum. Thus must
     * subtract off 1 byte - the size of the checksum - to get the 
     * index of the first element of the payload.
     */
	m_payload.setPayload(&sample[m_header.getSize() - 1]);
}

std::vector<unsigned char> UnknownIMUSample::getBytes()
{
	std::vector<unsigned char> vec(m_header.getMessageSize());

	for(int i = 0; i < m_header.getMessageSize(); ++i)
		vec[i] = m_bytes[i];

	return(vec);
}

bool UnknownIMUSample::isValidChecksum()
{
	std::vector<unsigned char> sampleChecksum(getBytes());
	std::vector<unsigned char> sampleNoChecksum(sampleChecksum.begin(), 
		--(sampleChecksum.end()));

	return(SensorDataUtils::isValidChecksum(sampleNoChecksum, 
		m_header.getChecksum()));
}

IIMUHeader * UnknownIMUSample::getHeader()
{
	return(&m_header);
}

std::vector<IIMUPayload *> UnknownIMUSample::getPayload()
{
	std::vector<IIMUPayload *> vec;

	vec.push_back(&m_payload);

	return(vec);
}

unsigned short UnknownIMUSample::getHeaderSize()
{
    return(m_header.getSize());
}

unsigned short UnknownIMUSample::getPayloadSize()
{
    return(m_payload.getSize());
}

unsigned short UnknownIMUSample::getSize()
{
    return(getHeaderSize() + getPayloadSize());
}

void UnknownIMUSample::setTransform(ISensorDataTransform * transform)
{
    m_transform = transform;
    m_payload.setTransform(m_transform);
}

ISensorDataTransform * UnknownIMUSample::getTransform()
{
    return(m_transform);
}
