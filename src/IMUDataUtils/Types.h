/*****************************************************************************
 * Types
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Standard types used to identify devices.
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
#ifndef MEMS_TYPES_H
#define MEMS_TYPES_H

namespace mems
{

enum E_DataFormatType
{
  BYTES, COUNTS, ENGINEERING_UNITS, VOLTS, DF_INVALID
};

enum E_DeviceType
{
  UIMU, UIMU_HIG, //2 axis
  UIMU_HIG_3AXES, //3 axis
  UIMU_TEMP, //1 temp
  UIMU_TEMP_3AXES, //3 temp
  UIMU_HIG_TEMP, //2 axis, 1 temp
  UIMU_HIG_TEMP_3AXES, //2 axis, 3 temp
  UIMU_HIG_3AXES_TEMP, //3 axis, 1 temp
  UIMU_HIG_3AXES_TEMP_3AXES, //3 axis, 3 temp
  NIMU, //1 temp
  NIMU_3TEMP, //3 temp
  BLUETOOTH, CDG, HARS, xAHRS, N2IMU, //1 temp
  N2IMU_3TEMP, //3 temp
  MG, //MAG3
  AR, //AccelRate
  TR, //TriRate
  UNKNOWN_IMU, DT_INVALID
};

enum E_ProtocolType
{
  RS422, I2C, PT_INVALID
};

} // mems

#endif // MEMS_TYPES_H
