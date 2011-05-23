#ifndef MEMS_SENSOR_DATA_TRANSFORM_DEFAULT_PRE
#define MEMS_SENSOR_DATA_TRANSFORM_DEFAULT_PRE

#include "ISensorDataTransform.h"

namespace mems
{

class SDTDefaultPre : public ISensorDataTransform
{
public:
   
    SDTDefaultPre(void);
    virtual ~SDTDefaultPre(void);

    short sample2Counts(unsigned char * sample);
    short sample2Counts(unsigned char msb, unsigned char lsb);

    double counts2EngUnits(short counts, double range);
    
    double counts2Celsius(short counts);
    double counts2Volts(short counts);

    double counts2TempVolts(short counts);
    double volts2Celsius( double volts );

    double celsius2Volts( double degrees );

private:
    static const double TEMP_RANGE;
    static const double TEMP_SENSITIVITY;
    static const double SENSOR_RANGE;
    static const double DIGITAL_SENSITIVITY;
    static const double TEMP_DIGITAL_SENSITIVITY;
};

} // mems

#endif // MEMS_SENSOR_DATA_TRANSFORM_DEFAULT_PRE
