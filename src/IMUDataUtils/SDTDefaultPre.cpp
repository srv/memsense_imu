/*****************************************************************************
 * SDTDefaultPre
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      This class cannot be instantiated and contains no data.  It is primarily
 *      used as a utility class to convert/process IMU data into 
 *      counts/engineering units/units.  Also has temp and checksum-specific methods.
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
#include "SDTDefaultPre.h"
#include "SensorDataUtils.h"

using namespace mems;

const double SDTDefaultPre::TEMP_RANGE = 5.0;//volts
const double SDTDefaultPre::TEMP_SENSITIVITY = 0.0084; //volts per degree_celsius
const double SDTDefaultPre::SENSOR_RANGE = 5.0; //in volts

//Convert from counts to volts:
//Represents the number of volts per bit:
const double SDTDefaultPre::DIGITAL_SENSITIVITY = SENSOR_RANGE / 32768.0;

//Convert temperature counts to volts:
//Represents degrees per bit:
const double SDTDefaultPre::TEMP_DIGITAL_SENSITIVITY = TEMP_RANGE / 32768.0 / TEMP_SENSITIVITY;


SDTDefaultPre::SDTDefaultPre(void)
{
}

SDTDefaultPre::~SDTDefaultPre(void)
{
}


short SDTDefaultPre::sample2Counts(unsigned char * sample)
{
    return(SensorDataUtils::sample2Counts(sample));
}

short SDTDefaultPre::sample2Counts(unsigned char msb, unsigned char lsb)
{
    return(SensorDataUtils::sample2Counts(msb, lsb));
}


double SDTDefaultPre::counts2EngUnits(short counts, double range)
{
   //Precoefficient counts cannot be converted to engineering units.
   return  -1;
}


double SDTDefaultPre::counts2Celsius(short counts)
{
    return (counts * TEMP_DIGITAL_SENSITIVITY) + 25;
}

double SDTDefaultPre::counts2Volts(short counts)
{
   return static_cast<double>(counts) * DIGITAL_SENSITIVITY;
}
   

double SDTDefaultPre::counts2TempVolts(short counts)
{
   return static_cast<double>(counts) * (TEMP_RANGE / 32768.0);
}


double SDTDefaultPre::volts2Celsius( double volts )
{
   return (volts / TEMP_SENSITIVITY + 25);
}

double SDTDefaultPre::celsius2Volts( double degrees )
{
   return ((degrees - 25) * TEMP_SENSITIVITY);
}

