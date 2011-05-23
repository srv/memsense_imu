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
#ifndef MEMS_COMMON_UTILS_H
#define MEMS_COMMON_UTILS_H

#include "Types.h"
#include <string>

namespace mems
{

// forward declarations
class IComm;
class IIMUHeader;
class SerialDeviceParams;
// class USBDeviceParams;


bool findSynch(IComm * comm, unsigned char * buffer, 
    const unsigned short numSynchBytes, const unsigned char synchByte,
    const unsigned short numMaxConsecutiveFailures = 10);

// NOTE: flushSerialBuffer is deprecated
void flushSerialBuffer(IComm * comm, unsigned int numBytes = 500);

bool flushBuffer(IComm * comm);
E_DeviceType findDevice(IIMUHeader & header);
std::string deviceType2Name(E_DeviceType device);
E_DeviceType name2DeviceType(std::string name);
std::string protocolType2Name(E_ProtocolType protocol);
E_ProtocolType name2ProtocolType(std::string name);
std::string dataFormatType2Name(E_DataFormatType dataFormat);

bool findDeviceSerial(SerialDeviceParams & params);
// bool findDeviceUSB(USBDeviceParams & params);
E_DeviceType deviceTypeFromHeaderDeviceID(unsigned short deviceID);

//Functions for determining the number of hiG/Temperature sensors:
int getNumberOfHiGAxes(E_DeviceType deviceType);
int getNumberOfTempSensors(E_DeviceType deviceType);

/**
* These functions are here for narrowing down devices based upon their packet
* size, once you know what type of device it is. The device type in the
* function name is assumed to be the MOST general that is allowed. For example:
* narrowDownUIMU can return any variation if the micro IMU, including all 
* combinations of numbers of HiG axes and temperature values; 
* narrowDownUIMUwHiG will only return micro IMUs that have HiG, but will allow
* the number of temperature values to fluctuate, and narrowDownUIMUw3HiG will
* return only devices with 3 hiG values, with slight variations for possible
* numbers of temperature values.
*
* This mean narrowDownUIMUwHiG possible return values are a subset of those
* from narrowDownUIMU, and a superset of those returned by narrowDownUIMUw3HiG.
**/
E_DeviceType narrowDownUIMU(unsigned short packetSize);
E_DeviceType narrowDownUIMUwHiG(unsigned short packetSize);
E_DeviceType narrowDownUIMUw3HiG(unsigned short packetSize);
E_DeviceType narrowDownUIMUwTemp(unsigned short packetSize);
E_DeviceType narrowDownUIMUw3Temp(unsigned short packetSize);
E_DeviceType narrowDownUIMUwTempAndHiG(unsigned short packetSize);
E_DeviceType narrowDownUIMUhiGW3Temp(unsigned short packetSize);
E_DeviceType narrowDownUnknownIMU(unsigned short packetSize);
E_DeviceType narrowDownUIMU3HiGwTemp(unsigned short packetSize);
E_DeviceType narrowDownNIMU(unsigned short packetSize);
E_DeviceType narrowDownN2IMU(unsigned short packetSize);

/**
* These are a series of constants designed to make the common utilities code
* easier to read. The packet size of each device type is defined here and used
* elsewhere in the code.
**/
static const unsigned short PACKETSIZE_UIMU = 32;
static const unsigned short PACKETSIZE_UIMU_1TEMP = 34;
static const unsigned short PACKETSIZE_UIMU_3TEMP = 38;
static const unsigned short PACKETSIZE_UIMU_2HIG = 36;
static const unsigned short PACKETSIZE_UIMU_3HIG = 38;
static const unsigned short PACKETSIZE_UIMU_1TEMP_2HIG = 38;
static const unsigned short PACKETSIZE_UIMU_1TEMP_3HIG = 40;
static const unsigned short PACKETSIZE_UIMU_3TEMP_2HIG = 42;
static const unsigned short PACKETSIZE_UIMU_3TEMP_3HIG = 44;
static const unsigned short PACKETSIZE_NIMU = 34;
static const unsigned short PACKETSIZE_NIMU_3TEMP = 38;
static const unsigned short PACKETSIZE_N2IMU = 34;
static const unsigned short PACKETSIZE_N2IMU_3TEMP = 38;
static const unsigned short PACKETSIZE_BLUETOOTH = 38;


} // mems

#endif // MEMS_COMMON_UTILS_H
