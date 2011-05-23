#include "DeviceParams.h"

using namespace mems;

DeviceParams::DeviceParams()
{

}

DeviceParams::DeviceParams
(
    int numSynchBytes, 
    unsigned char synchByte, 
    int sampleSize, 
    E_DeviceType devType, 
    E_ProtocolType protocol
)
{
    m_numSynchBytes = numSynchBytes;
    m_synchByte = synchByte;
    m_sampleSize = sampleSize;
    m_device = devType;
    m_protocol = protocol;
}

DeviceParams::~DeviceParams()
{
    m_numSynchBytes = 0;
    m_synchByte = 0xFF;
}

void DeviceParams::setNumSynchBytes(int numSynchBytes)
{
    m_numSynchBytes = numSynchBytes;
}

int DeviceParams::getNumSynchBytes()
{
    return(m_numSynchBytes);
}

void DeviceParams::setSynchByte(unsigned char synchByte)
{
    m_synchByte = synchByte;
}

unsigned char DeviceParams::getSynchByte()
{
    return(m_synchByte);
}

void DeviceParams::setSampleSize(int size)
{
    m_sampleSize = size;
}

int DeviceParams::getSampleSize()
{
    return(m_sampleSize);
}

void DeviceParams::setDeviceType(E_DeviceType type)
{
    m_device = type;
}

E_DeviceType DeviceParams::getDeviceType()
{
    return(m_device);
}

void DeviceParams::setProtocol(E_ProtocolType type)
{
    m_protocol = type;
}

E_ProtocolType DeviceParams::getProtocol()
{
    return(m_protocol);
}
