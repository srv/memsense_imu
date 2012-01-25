/*****************************************************************************
 * GlobalVars
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Global variable declarations used primarily by menu system and
 *          data output.
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
 *   6/06/06: Added g_baud rate for setting of baud rate.
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_GLOBAL_VARS_H
#define MEMS_GLOBAL_VARS_H

#include <string>
#include "IMUDataUtils/Types.h"

namespace mems
{

enum DataConversion
{
  PRE_COEFF = 0, POST_COEFF
};

extern bool g_printCounter; // true if counter printed to console/file
extern bool g_printConsole; // true if should print to console
extern bool g_printHeader; // true if should print header data (bytes only)
extern bool g_consoleTempCelsius; // true if should print temp to console in Celsius
extern int g_printConsoleN; // every n lines printed to console
extern int g_consoleSampleCount; // counter for printing to console
extern double g_gyroRange; // range for gyro
extern double g_accelRange; // range for accel
extern double g_magRange; // range for mag
extern double g_higRange; // range for hi-g accel (uimu)
extern std::string g_ofilename; // output filename
extern std::string g_comPort; // COM port to receive data (RS422)
extern int g_baudRate; // COM port baud rate (RS422)
extern short g_deviceIndex; // USB device index (I2C)
extern E_DataFormatType g_dataFormat; // type of data format to output
extern E_ProtocolType g_protocol; // type of protocol
extern E_DeviceType g_device; // type of device
extern std::string g_idcVersion; // version of IDC
extern int g_COMStart; // COM port to start auto-config search
extern int g_COMStop; // COM port to end auto-config search
extern int g_sampleSize; // size of full sample

extern DataConversion g_conversion;

} // mems

#endif // MEMS_GLOBAL_VARS_H
