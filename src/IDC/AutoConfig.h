/*****************************************************************************
 * AutoConfig
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Attempts to automatically determine the correct configuration of
 *          the application.  Rolls across the COM/USB ports to find
 *          IMU data and, if found, uses the size of message to determine
 *          type of IMU.
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
#ifndef MEMS_AUTO_CONFIG_H
#define MEMS_AUTO_CONFIG_H

#include "IMUDataUtils/Types.h"

namespace mems
{

bool autoConfig();
bool findDeviceSerial();
// bool findDeviceUSB();

} // mems

#endif // MEMS_AUTO_CONFIG_H
