/*****************************************************************************
 * IIMUHeader
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Interface that must be implemented by all classes for new IMU data
 *      headers.  If not implemented, will not be available for use within
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

#ifndef MEMS_IIMUHEADER_H
#define MEMS_IIMUHEADER_H

#include <vector>

namespace mems
{

class IComm;

class IIMUHeader
{
public:

  virtual unsigned short getMessageSize() const = 0;
  virtual unsigned short getDeviceID() const = 0;
  virtual unsigned short getMessageID() const = 0;
  virtual unsigned char getChecksum() const = 0;
  virtual unsigned short getCounter() const = 0;
  virtual std::vector<unsigned char> getBytes() = 0;
  virtual unsigned short getSize() = 0;
};

}

#endif // MEMS_IIMUHEADER_H
