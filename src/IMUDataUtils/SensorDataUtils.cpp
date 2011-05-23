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
#include "SensorDataUtils.h"

using namespace mems;

const double SensorDataUtils::TEMP_RANGE = 5.0;
const double SensorDataUtils::TEMP_SENSITIVITY = 0.0084;
const double SensorDataUtils::PI = 3.141592653589793238;

SensorDataUtils::SensorDataUtils(void)
{
}

SensorDataUtils::~SensorDataUtils(void)
{
}

short SensorDataUtils::sample2Counts(unsigned char * sample)
{
    /**
     * each device-specific sample 
     * (e.g. gyro) is a 16-bit signed integer.  Pull out 
     * each device sample by left shifting byte 0 and
     * biwise OR-ing with byte 1 - producing a single
     * 16-bit value.
     **/
    return((sample[0] << 8) + sample[1]);
}

short SensorDataUtils::sample2Counts(unsigned char msb, unsigned char lsb)
{
    /**
     * Per IMU documentation, each device-specific sample 
     * (e.g. gyro) is a 16-bit signed integer.  Pull out 
     * each device sample by left shifting byte 0 (msb) and
     * biwise OR-ing with byte 1 (lsb) - producing a single
     * 16-bit value.
     **/
    return((msb << 8) + lsb);
}

double SensorDataUtils::counts2Celsius(short counts)
{
    return ((1.5 * counts/32768) * TEMP_RANGE);
}

unsigned short SensorDataUtils::getMSB(unsigned short val)
{
    return(val >> 8);
}

unsigned short SensorDataUtils::getLSB(unsigned short val)
{
    return(val & 0x00ff);
}

bool SensorDataUtils::isValidChecksum(std::vector<unsigned char> & sample, short checksum)
{
    bool isValid = false;
    int sum = 0;
	std::vector<unsigned char>::iterator iter;

    /**
     * summing of all data in sample - except checksum block -
     * should equal checksum value.  
     * 
     * NOTE: Checksum calculation is found under 'Sample Format' section
     * in IMU docs.  Also note that this is done by working with the 'mod' 
     * function - is not necessarily a direct addition unless working
     * with 8-bit type.
     **/
    for(iter = sample.begin(); iter != sample.end(); ++iter)
        sum += *iter;

    if(sum % 256 == checksum)
        isValid = true;

    return(isValid);
}

double SensorDataUtils::deg2rad(double deg)
{
    return(deg * PI/180.0);
}

double SensorDataUtils::rad2deg(double rad)
{
    return(rad * 180.0/PI);
}
