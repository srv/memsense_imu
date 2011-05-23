#ifndef MEMS_ISENSOR_DATA_TRANSFORM_H
#define MEMS_ISENSOR_DATA_TRANSFORM_H

namespace mems
{

class ISensorDataTransform
{
public:

    virtual short sample2Counts(unsigned char * sample) = 0;
    virtual short sample2Counts(unsigned char msb, unsigned char lsb) = 0;

    virtual double counts2Volts(short counts) = 0;

    virtual double counts2EngUnits(short counts, double range) = 0;
    virtual double counts2Celsius(short counts) = 0;

    virtual double counts2TempVolts(short counts) = 0;
    virtual double volts2Celsius( double volts ) = 0;

    virtual double celsius2Volts( double degrees ) = 0;

protected:
    ISensorDataTransform(){};
    
};

} // mems

#endif // MEMS_ISENSOR_DATA_TRANSFORM_H