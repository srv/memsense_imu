/*****************************************************************************
 * SDTDefault
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
#include "SDTDefault.h"
#include "SensorDataUtils.h"

using namespace mems;

const double SDTDefault::TEMP_RANGE = 5.0; //Volts
const double SDTDefault::TEMP_SENSITIVITY = 0.0084; //Volts / degree_celsius
const double SDTDefault::SENSOR_RANGE = 5.0; //in volts

//Convert from counts to volts:
//Represents the number of volts per bit:
const double SDTDefault::DIGITAL_SENSITIVITY = SENSOR_RANGE / 32768.0;

//Convert temperature counts to volts:
//Represents degrees per bit:
const double SDTDefault::TEMP_DIGITAL_SENSITIVITY = TEMP_RANGE / 32768.0 / TEMP_SENSITIVITY;


SDTDefault::SDTDefault(void)
{
}

SDTDefault::~SDTDefault(void)
{
}

short SDTDefault::sample2Counts(unsigned char * sample)
{
    return(SensorDataUtils::sample2Counts(sample));
}

short SDTDefault::sample2Counts(unsigned char msb, unsigned char lsb)
{
    return(SensorDataUtils::sample2Counts(msb, lsb));
}

double SDTDefault::counts2EngUnits(short counts, double range)
{
    return (((1.5 * static_cast<double>(counts)) / 32768.0) * range);
}


double SDTDefault::counts2Celsius(short counts)
{
   return (counts * TEMP_DIGITAL_SENSITIVITY) + 25;
}

double SDTDefault::counts2Volts(short counts)
{
   //Post coefficient cannot return the actual voltage.
   //The user shouldn't be calling this function, so they
   //get an error return.
   return -1.0;
}

double SDTDefault::counts2TempVolts(short counts)
{
   return static_cast<double>(counts) * (TEMP_RANGE / 32768.0);
}


double SDTDefault::volts2Celsius( double volts )
{
   return (volts / TEMP_SENSITIVITY + 25);
}

double SDTDefault::celsius2Volts( double degrees )
{
   return ((degrees - 25) * TEMP_SENSITIVITY);
}


