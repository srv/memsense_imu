/*****************************************************************************
 * SensorDataUtils
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      This class cannot be instantiated and contains no data.  It is primarily
 *      used as a utility class to convert/process IMU data into 
 *      counts/EngUnits/units.  Also has temp and checksum-specific methods.
 *      Note that all methods of this class are static.
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
#ifndef MEMS_SENSORDATAUTILS_H
#define MEMS_SENSORDATAUTILS_H

#include <vector>

namespace mems
{

class SensorDataUtils
{
public:

  //5.0 * counts/32768 - eng units comes out as volts.
  //4 Removed:

  static double counts2Celsius(short counts);
  static short sample2Counts(unsigned char * sample);
  static short sample2Counts(unsigned char msb, unsigned char lsb);

  static unsigned short getMSB(unsigned short val);
  static unsigned short getLSB(unsigned short val);

  static bool isValidChecksum(std::vector<unsigned char> & sample, short checksum);

  static double deg2rad(double deg);
  static double rad2deg(double rad);

private:

  static const double TEMP_RANGE;
  static const double TEMP_SENSITIVITY;
  static const double PI;

  SensorDataUtils(void); // class is not instantiable
  ~SensorDataUtils(void);
};

} // mems

#endif // MEMS_SENSORDATAUTILS_H
