#ifndef MEMS_DEVICE_PARAMS_H
#define MEMS_DEVICE_PARAMS_H

#include "Types.h"

namespace mems
{

class DeviceParams
{
public:

    DeviceParams();
    DeviceParams(int numSynchBytes, unsigned char synchByte, int sampleSize, 
        E_DeviceType devType, E_ProtocolType protocol);
    virtual ~DeviceParams();

    void setNumSynchBytes(int numSynchBytes);
    int getNumSynchBytes();

    void setSynchByte(unsigned char synchByte);
    unsigned char getSynchByte();

    void setSampleSize(int size);
    int getSampleSize();

    void setDeviceType(E_DeviceType type);
    E_DeviceType getDeviceType();

    void setProtocol(E_ProtocolType type);
    E_ProtocolType getProtocol();

private:
    int             m_numSynchBytes;
    unsigned char   m_synchByte;
    E_DeviceType    m_device;
    E_ProtocolType  m_protocol;
    int             m_sampleSize;
};

} // mems

#endif // MEMS_DEVICE_PARAMS_H