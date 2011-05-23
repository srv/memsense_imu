/*****************************************************************************
 * IIMUSample
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Interface that must be implemented by all classes implementing new
 *      sample structure.  If not implemented, will not be available for use 
 *      within current architecture.
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
#ifndef MEMS_IIMUSAMPLE_H
#define MEMS_IIMUSAMPLE_H

#include <vector>

namespace mems
{

class IComm;
class IIMUPayload;
class IIMUHeader;
class ISensorDataTransform;

class IIMUSample
{
public:
    virtual bool readSample(IComm * comm,
                         int & status,
                         const unsigned short findSynchMaxConsecutiveFailures = 10) = 0;
    virtual void setSample(unsigned char * sample) = 0;
	virtual std::vector<unsigned char> getBytes() = 0;
	virtual IIMUHeader * getHeader() = 0;
    virtual unsigned short getHeaderSize() = 0;
	virtual std::vector<IIMUPayload *> getPayload() = 0;
    virtual unsigned short getPayloadSize()= 0;
    virtual bool isValidChecksum() = 0;
    virtual unsigned short getSize() = 0;
    virtual IIMUSample * clone() = 0;
    virtual void setTransform(ISensorDataTransform * transform) = 0;
    virtual ISensorDataTransform * getTransform() = 0;
};

}

#endif // MEMS_IIMUSAMPLE_H