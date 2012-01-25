/*****************************************************************************
 * GlobalVars
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Global variable definitions used primarily by menu system and
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
 *   1/26/07: Added g_COMStop/g_COMStart for setting range of auto-config search
 *   1/26/07: Added g_sampleSize
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/

#include "GlobalVars.h"

namespace mems
{

bool g_printCounter = false; // true if counter printed to console/file
bool g_printConsole = true; // true if should print to console
bool g_printHeader = false; // true if should print header data (bytes only)
bool g_consoleTempCelsius = true;// true if should print temp to console in Celsius
int g_printConsoleN = 20; // every n lines printed to console
int g_consoleSampleCount = 0; // counter for printing to console
double g_gyroRange = 300.0; // range for gyro
double g_accelRange = 5.0; // range for accel
double g_magRange = 1.9; // range for mag
double g_higRange = 5.0; // range for hi-g accel (uimu)
std::string g_ofilename; // output filename
std::string g_comPort = "/dev/ttyUSB0"; // COM port to receive data (RS422)
int g_baudRate = 115200; // COM port baud rate (RS422)
short g_deviceIndex = 0; // USB device index (I2C)
E_DataFormatType g_dataFormat = ENGINEERING_UNITS;//type of data format to output
E_ProtocolType g_protocol = RS422; // type of protocol
E_DeviceType g_device = UIMU; // type of device
std::string g_idcVersion = "1.1.0.I"; // current version
int g_COMStart = 0; // COM port to start auto-config search
int g_COMStop = 10; // COM port to end auto-config search
int g_sampleSize = 0; // size of sample read from device


DataConversion g_conversion = PRE_COEFF;

} // mems
