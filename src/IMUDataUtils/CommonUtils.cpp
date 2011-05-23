/*****************************************************************************
 * CommonUtils
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Common utility functions.
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
 *   3/14/06: Initial Release
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/

#include "CommonUtils.h"
#include "IComm.h"
#include "StandardIMUHeader.h"
#include "UnknownIMUSample.h"
#include "SerialDeviceParams.h"
// #include "USBDeviceParams.h"
// #include "USBComm.h"
#include "STDIMUSample.h"
#include <iostream>
#include <sstream>  // ostringstream


namespace mems
{


static const short SAMPLE_SIZE_UIMU = 32;
static const short SAMPLE_SIZE_UIMU_HIG = 36;
static const short SAMPLE_SIZE_UIMU_HIG_3AXES = 38;
static const short SAMPLE_SIZE_UIMU_TEMP = 38;
static const short SAMPLE_SIZE_NIMU = 38;
static const short SAMPLE_SIZE_UIMU_HIG_TEMP = 42;
static const short SAMPLE_SIZE_UIMU_HIG_3AXES_TEMP = 44;
static const short SAMPLE_SIZE_NO_IMU_MATCH = 0;

// Device constants
static const char * STR_UIMU = "uIMU";
static const char * STR_UIMU_HIG = "uIMU Hi-G";
static const char * STR_UIMU_TEMP = "uIMU w/ Temp";
static const char * STR_UIMU_HIG_TEMP = "uIMU w/ Hi-G and Temp";
static const char * STR_NIMU = "nIMU";
static const char * STR_UNKNOWN_IMU = "Unknown IMU";
static const char * STR_BLUETOOTH = "BlueTooth";
static const char * STR_CDG = "CDG";
static const char * STR_HARS = "HARS";
static const char * STR_xAHRS = "xAHRS";
static const char * STR_N2IMU = "N2IMU";
static const char * STR_AR = "AR";
static const char * STR_MG = "MG";
static const char * STR_TR = "TR";

// Protocol constants
static const char * STR_RS422 = "RS-422";
static const char * STR_I2C = "I2C";
static const char * STR_INVALID = "Invalid";

// number of IComm read attempts
static const short READ_RETRIES = 2;

bool findDeviceCommon(IComm & comm, DeviceParams & params);

bool readDataRetry(IComm & comm, unsigned long bytesToRead, unsigned char * buffer, int numRetries,
                   unsigned long & numBytesRead, int & status);

} // mems

///////////////////////////////////////////////////////////////////////////////
// using namespace mems;
///////////////////////////////////////////////////////////////////////////////

mems::E_DeviceType mems::deviceTypeFromHeaderDeviceID(unsigned short deviceID)
{
  switch (deviceID)
  {
    case 1:
      return UIMU;

    case 2:
      return UIMU_HIG_3AXES;

    case 3:
      return NIMU;

    case 4:
      return BLUETOOTH;

    case 5:
      return CDG;

    case 6:
      return HARS;

    case 7:
      return xAHRS;

    case 8:
      return N2IMU;

      //Older devices don't return an ID, return invalid. Also return invalid
      //if we got a value we don't recognize:
    case 0:
    case 9:
    default:
      return DT_INVALID;
  }
}

int mems::getNumberOfHiGAxes(E_DeviceType deviceType)
{
  switch (deviceType)
  {
    //Very few have any hiG sensors:
    default:
      return 0;

    case UIMU_HIG:
    case UIMU_HIG_TEMP:
    case UIMU_HIG_TEMP_3AXES:
      return 2;

    case UIMU_HIG_3AXES:
    case UIMU_HIG_3AXES_TEMP:
    case UIMU_HIG_3AXES_TEMP_3AXES:
      return 3;
  }
}

int mems::getNumberOfTempSensors(E_DeviceType deviceType)
{
  switch (deviceType)
  {
    default:
      return 0;

    case UIMU_TEMP:
    case UIMU_HIG_TEMP:
    case UIMU_HIG_3AXES_TEMP:
    case NIMU:
    case N2IMU:
      return 1;

    case UIMU_TEMP_3AXES:
    case UIMU_HIG_TEMP_3AXES:
    case UIMU_HIG_3AXES_TEMP_3AXES:
    case NIMU_3TEMP:
    case BLUETOOTH:
    case N2IMU_3TEMP:
      return 3;
  }
}

mems::E_DeviceType mems::narrowDownUIMU(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU:
      return UIMU;

    case PACKETSIZE_UIMU_1TEMP:
      return UIMU_TEMP;

    case PACKETSIZE_UIMU_3TEMP:
      return UIMU_TEMP_3AXES;

    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUIMUwHiG(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_2HIG:
      return UIMU_HIG;

    case PACKETSIZE_UIMU_3HIG:
      return UIMU_HIG_3AXES;

    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}
mems::E_DeviceType mems::narrowDownUIMUw3HiG(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_3HIG:
      return UIMU_HIG_3AXES;

    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}
mems::E_DeviceType mems::narrowDownUIMUwTemp(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_1TEMP:
      return UIMU_TEMP;

    case PACKETSIZE_UIMU_3TEMP:
      return UIMU_TEMP_3AXES;

    case PACKETSIZE_UIMU_3TEMP_2HIG:
      return UIMU_HIG_TEMP_3AXES;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUIMUwTempAndHiG(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_1TEMP:
      return UIMU_TEMP;

    case PACKETSIZE_UIMU_1TEMP_2HIG:
      return UIMU_HIG_TEMP;

    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUIMUhiGW3Temp(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_3TEMP_2HIG:
      return UIMU_HIG_TEMP_3AXES;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUnknownIMU(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU:
      return UIMU;

    case PACKETSIZE_NIMU:
      return NIMU;

    case PACKETSIZE_NIMU_3TEMP:
      return NIMU_3TEMP;

    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUIMUw3Temp(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_3TEMP:
      return UIMU_HIG_TEMP_3AXES;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownUIMU3HiGwTemp(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_UIMU_1TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP;

    case PACKETSIZE_UIMU_3TEMP_3HIG:
      return UIMU_HIG_3AXES_TEMP_3AXES;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownNIMU(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_NIMU:
      return NIMU;

    case PACKETSIZE_NIMU_3TEMP:
      return NIMU_3TEMP;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::narrowDownN2IMU(unsigned short packetSize)
{
  switch (packetSize)
  {
    case PACKETSIZE_N2IMU:
      return N2IMU;

    case PACKETSIZE_N2IMU_3TEMP:
      return N2IMU_3TEMP;

    default:
      return DT_INVALID;
  }
}

mems::E_DeviceType mems::findDevice(mems::IIMUHeader & header)
{
  E_DeviceType devType = deviceTypeFromHeaderDeviceID(header.getDeviceID());
  unsigned short size = header.getMessageSize();

  //Now, based upon the best information on the device type that we have,
  //we can check the size of the messages to narrow it down further:
  switch (devType)
  {
    case UIMU:
      return narrowDownUIMU(size);

    case UIMU_HIG:
      return narrowDownUIMUwHiG(size);

    case UIMU_HIG_3AXES:
      return narrowDownUIMUw3HiG(size);

    case UIMU_TEMP:
      return narrowDownUIMUwTemp(size);

    case UIMU_TEMP_3AXES:
      return narrowDownUIMUw3Temp(size);

    case UIMU_HIG_TEMP:
      return narrowDownUIMUwTempAndHiG(size);

    case UIMU_HIG_TEMP_3AXES:
      return narrowDownUIMUhiGW3Temp(size);

    case UIMU_HIG_3AXES_TEMP:
      return narrowDownUIMU3HiGwTemp(size);

      //Already narrowed down to 3 axes, just ensure the proper packet size:
    case UIMU_HIG_3AXES_TEMP_3AXES:
      if (size == PACKETSIZE_UIMU_3TEMP_3HIG)
        return UIMU_HIG_3AXES_TEMP_3AXES;
      else
        return DT_INVALID;
      break;

    case NIMU:
      return narrowDownNIMU(size);

      //Know we have 3 temperatures, make sure of the right packet size:
    case NIMU_3TEMP:
      if (size == PACKETSIZE_NIMU_3TEMP)
        return NIMU_3TEMP;
      else
        return DT_INVALID;
      break;

      //Only one possible option. Just make sure its the right size:
    case BLUETOOTH:
      if (size == PACKETSIZE_BLUETOOTH)
        return BLUETOOTH;
      else
        return DT_INVALID;
      break;

      //No checks to make:
    case CDG:
    case HARS:
    case xAHRS:
      return devType;

    case N2IMU:
      return narrowDownN2IMU(size);

      //Just ensure the proper size:
    case N2IMU_3TEMP:
      if (size == PACKETSIZE_N2IMU_3TEMP)
        return N2IMU_3TEMP;
      else
        return DT_INVALID;
      break;

    case MG:
    case AR:
    case TR:
      return devType;
      break;

    case UNKNOWN_IMU:
    case DT_INVALID:
    default:
      return narrowDownUnknownIMU(size);
      break;
  }
}

std::string mems::deviceType2Name(mems::E_DeviceType device)
{
  std::string name;

  switch (device)
  {
    case UIMU:
      name = STR_UIMU;
      break;

    case UIMU_HIG:
    case UIMU_HIG_3AXES:
      name = STR_UIMU_HIG;
      break;

    case UIMU_TEMP_3AXES:
    case UIMU_TEMP:
      name = STR_UIMU_TEMP;
      break;

    case UIMU_HIG_TEMP:
    case UIMU_HIG_3AXES_TEMP:
    case UIMU_HIG_TEMP_3AXES:
    case UIMU_HIG_3AXES_TEMP_3AXES:
      name = STR_UIMU_HIG_TEMP;
      break;

    case NIMU:
    case NIMU_3TEMP:
      name = STR_NIMU;
      break;

    case BLUETOOTH:
      name = STR_BLUETOOTH;
      break;

    case CDG:
      name = STR_CDG;
      break;

    case HARS:
      name = STR_HARS;
      break;

    case xAHRS:
      name = STR_xAHRS;
      break;

    case N2IMU:
      name = STR_N2IMU;
      break;

    case UNKNOWN_IMU:
      name = STR_UNKNOWN_IMU;
      break;

    case AR:
      name = STR_AR;
      break;

    case TR:
      name = STR_TR;
      break;

    case MG:
      name = STR_MG;
      break;

    default:
      name = STR_INVALID;
      break;
  }

  return (name);
}

mems::E_DeviceType mems::name2DeviceType(std::string name)
{
  E_DeviceType type = DT_INVALID;

  if (name == STR_UIMU)
    type = UIMU;
  else if (name == STR_UIMU_HIG)
    type = UIMU_HIG;
  else if (name == STR_UIMU_TEMP)
    type = UIMU_TEMP;
  else if (name == STR_UIMU_HIG_TEMP)
    type = UIMU_HIG_TEMP;
  else if (name == STR_NIMU)
    type = NIMU;
  else if (name == STR_BLUETOOTH)
    type = BLUETOOTH;
  else if (name == STR_CDG)
    type = CDG;
  else if (name == STR_HARS)
    type = HARS;
  else if (name == STR_xAHRS)
    type = xAHRS;
  else if (name == STR_N2IMU)
    type = N2IMU;
  else if (name == STR_UNKNOWN_IMU)
    type = UNKNOWN_IMU;
  else if (name == STR_AR)
    type = AR;
  else if (name == STR_MG)
    type = MG;
  else if (name == STR_TR)
    type = TR;

  return (type);
}

std::string mems::protocolType2Name(mems::E_ProtocolType protocol)
{
  std::string name;

  switch (protocol)
  {
    case I2C:
      name = STR_I2C;
      break;

    case RS422:
      name = STR_RS422;
      break;

    default:
      name = STR_INVALID;
      break;
  }

  return (name);
}

mems::E_ProtocolType mems::name2ProtocolType(std::string name)
{
  E_ProtocolType type = PT_INVALID;

  if (name == STR_I2C)
    type = I2C;
  else if (name == STR_RS422)
    type = RS422;

  return (type);
}

std::string mems::dataFormatType2Name(mems::E_DataFormatType dataFormat)
{
  std::string name;

  switch (dataFormat)
  {
    case BYTES:
      name = "Bytes";
      break;

    case COUNTS:
      name = "Counts";
      break;

    case ENGINEERING_UNITS:
      name = "Engineering Units";
      break;

    case VOLTS:
      name = "Volts";
      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }

  return (name);
}

bool mems::findSynch(mems::IComm * comm, unsigned char * buffer, const unsigned short numSynchBytes,
                     const unsigned char synchByte, const unsigned short numMaxConsecutiveFailures)
{
  int synchCount = 0;
  bool success = false;
  unsigned long bytesRead;
  static const short BYTES_TO_READ = 1;
  static const short MAX_ATTEMPTS = 200;
  int status;
  int consecReadFails = 0;
  int i = 0;

  /**
   * Pretty straightforward - find the synch bytes, ignoring all other
   * data until found.  Note that if a series of consecutive read failures
   * occurs the loop will bail with the function returning false (FAIL).
   **/
  while (synchCount < numSynchBytes && i < MAX_ATTEMPTS)
  {
    if (readDataRetry(*comm, BYTES_TO_READ, buffer + synchCount, READ_RETRIES, bytesRead, status))
    {
      // read a byte, so reset tracking of consecutive read failures
      consecReadFails = 0;

      // determine if synch byte was read
      if (bytesRead == 1 && buffer[synchCount] == synchByte)
        synchCount++;
      else
        synchCount = 0;

      if (synchCount == numSynchBytes)
        success = true;
    }
    else if (++consecReadFails == numMaxConsecutiveFailures)
      break;

    ++i;
  }

  return (success);
}

void mems::flushSerialBuffer(mems::IComm * comm, unsigned int numBytes)
{
  static const unsigned short MAX_READ_FAILS = 10;
  unsigned char* temp_buffer = new unsigned char[numBytes];
  unsigned long bytes_read = 0;
  unsigned long bytes_this_time;
  short failCount = 0;
  int status;

  //read in, and throw away number of bytes passed in.
  do
  {
    if (comm->readData(numBytes, temp_buffer, bytes_this_time, status))
    {
      bytes_read += bytes_this_time;
      failCount = 0;
    }
    else
      ++failCount;

  } while (bytes_read < numBytes && failCount < MAX_READ_FAILS);
}

bool mems::flushBuffer(mems::IComm * comm)
{
  int status;

  return (comm->flushBuffer(status));
}

bool mems::findDeviceSerial(SerialDeviceParams & params)
{
  SerialComm comm;
  SerialComm::SDeviceData devData;

  int status;
  bool success = false;

  // specify port to open
  std::ostringstream ost;
  ost << "/dev/ttyUSB" << params.getPort();
  devData.m_comPort = ost.str();

  if (comm.openDevice(&devData, status))
  {
    comm.setBaudRate(params.getBaudRate());
    comm.setDataBits(params.getDataBits());
    comm.setStopBits(params.getStopBits());
    comm.setParity(params.getParity());
    comm.setReadTimeout(params.getReadTimeout());

    if (findDeviceCommon(comm, params))
    {
      params.setProtocol(RS422);
      success = true;
    }
  }

  // ensure the previously opened device is closed
  comm.closeDevice(status);

  return (success);
}

//bool mems::findDeviceUSB(USBDeviceParams & params)
//{
//    using namespace mems;
//
//    USBComm comm;
//    USBComm::SDeviceData devData;
//
//    int status;
//    bool success = false;
//    devData.m_deviceIndex = params.getDeviceIndex();
//    if(comm.openDevice(&devData, status))
//    {
//        comm.setTimeouts(params.getReadTimeout(), params.getWriteTimeout(), status);
//
//        if(findDeviceCommon(comm, params))
//        {
//            params.setProtocol(I2C);
//            success = true;
//        }
//    }
//
//
//    return(success);
//}


bool mems::findDeviceCommon(IComm & comm, DeviceParams & params)
{
  IIMUHeader* header;
  E_DeviceType devType = DT_INVALID;
  unsigned char headerBuffer[StandardIMUHeader::HEADER_PLUS_CHECKSUM_SIZE];
  unsigned char * sampleBuffer = NULL;
  unsigned long bytesRead = 0;
  bool success = false;
  int checksum = 0;
  int status;
  unsigned int size;
  STDIMUSample* sample = NULL;

  // it was flushSerialBuffer(&comm) -- Joan Pau
  flushBuffer(&comm);

  // find the synch bytes so that the next byte read in is the size
  if (findSynch(&comm, headerBuffer, params.getNumSynchBytes(), params.getSynchByte()))
  {
    /**
     * read in the size of the packet and allocate a buffer large
     * enough to contain the data
     */
    readDataRetry(comm, 1, headerBuffer, READ_RETRIES, bytesRead, status);
    size = static_cast<unsigned int> (headerBuffer[0]);

    /**
     * some basic attempt to determine if a valid packet.  Should at least
     * be larger than a full header/checksum.
     **/
    if (size > StandardIMUHeader::HEADER_PLUS_CHECKSUM_SIZE)
    {
      sampleBuffer = new unsigned char[size];

      // read in size bytes (synch bytes already read in, as is size byte)
      if (readDataRetry(comm, size - params.getNumSynchBytes() - 1, sampleBuffer, READ_RETRIES, bytesRead, status))
      {
        /**
         * calculate the checksum
         * NOTE: don't include sample checksum in summation
         **/
        for (unsigned int i = 0; i < bytesRead - 1; ++i)
          checksum += sampleBuffer[i];

        // count the size in the checksum
        checksum += size;

        // count the synch bytes
        checksum += (params.getSynchByte() * params.getNumSynchBytes());
        checksum %= 256;

        // valid checksum?
        if (checksum == static_cast<int> (sampleBuffer[bytesRead - 1]))
        {
          sample = STDIMUSample::getStandardSample(static_cast<unsigned short> (sampleBuffer[0]), size); //leave default sensor ranges.

          // read in a sample...
          if (sample->readSample(&comm, status))
          {
            // check the checksum...again...
            if (sample->isValidChecksum())
            {
              header = sample->getHeader();
              devType = findDevice(*header);

              if (devType != DT_INVALID)
              {
                params.setDeviceType(devType);
                params.setSampleSize(header->getMessageSize());
                success = true;
              }
            }
          }

          delete sample;

        }// ValidChecksum == true

      }// readData

      // cleanup
      delete[] sampleBuffer;

    }// validate size
  }// findSynch

  return (success);
}

bool mems::readDataRetry(IComm & comm, unsigned long bytesToRead, unsigned char * buffer, int numRetries,
                         unsigned long & numBytesRead, int & status)
{
  unsigned long bytes_this_time;
  short failCount = 0;

  numBytesRead = 0;

  do
  {
//    if (comm.readData(bytesToRead - numBytesRead, buffer, bytes_this_time, status))
    comm.readData(bytesToRead - numBytesRead, buffer + numBytesRead, bytes_this_time, status);
    if (bytes_this_time)
    {
      numBytesRead += bytes_this_time;
      failCount = 0;
    }
    else
      ++failCount;

  } while (numBytesRead < bytesToRead && failCount < numRetries);

  return (numBytesRead == bytesToRead);
}
