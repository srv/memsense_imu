/*****************************************************************************
 * IComm
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Interface class that enforces standard methods available across
 *      all communications classes.  All communications classes *must*
 *      derive from this interface in order to be integrated within
 *      existing application architectures.
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
#ifndef MEMS_ICOMM_H
#define MEMS_ICOMM_H

namespace mems
{

class IComm
{
public:
    virtual ~IComm(){}
    virtual bool openDevice(void * data, int & status) = 0;
    virtual bool closeDevice(int & status) = 0;
    virtual bool readData(unsigned long numBytesToRead, 
                    unsigned char * buffer, unsigned long & numBytesRead, 
                    int & status) = 0;
    virtual bool writeData(unsigned long numBytesToWrite, unsigned char * buffer, 
                    unsigned long & numBytesWritten, int & status) = 0;
    virtual bool flushBuffer(int & status) = 0;
};

} // mems

#endif // MEMS_ICOMM_H