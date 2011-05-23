/*****************************************************************************
 * IIMUPayload
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Interface that must be implemented by all classes for new IMU data
 *      payloads.  If not implemented, will not be available for use within
 *      current architecture.
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
#ifndef MEMS_IIMUPAYLOAD_H
#define MEMS_IIMUPAYLOAD_H

#include <vector>

namespace mems
{

class ISensorDataTransform;

class IIMUPayload
{
public:

    virtual std::vector<unsigned char> getBytes() = 0;
    virtual std::vector<short> getCounts() = 0;
    virtual std::vector<double> getEngUnits() = 0;
    virtual std::vector<double> getVolts() = 0;
    virtual unsigned short getSize() = 0;

    virtual void setTransform(ISensorDataTransform * transform);
    virtual ISensorDataTransform * getTransform();

protected:
    IIMUPayload();
    IIMUPayload(ISensorDataTransform * transform);
    virtual ~IIMUPayload();

private:
    ISensorDataTransform  * m_sdt;
};

} // mems

#endif // MEMS_IIMUPAYLOAD_H
