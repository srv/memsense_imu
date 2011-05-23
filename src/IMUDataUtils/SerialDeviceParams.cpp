#include "SerialDeviceParams.h"

using namespace mems;

SerialDeviceParams::SerialDeviceParams()
{

}

SerialDeviceParams::SerialDeviceParams
(
    int port,
    int baudRate,
    SerialComm::E_DataBits dataBits,
    SerialComm::E_StopBits stopBits,
    SerialComm::E_Parity parity,
    int readTimeout
)
{
    m_port = port;
    m_dataBits = dataBits;
    m_baudRate = baudRate;
    m_stopBits = stopBits;
    m_parity = parity;
    m_readTimeout = readTimeout;
}

SerialDeviceParams::~SerialDeviceParams()
{

}

void SerialDeviceParams::setPort(int port)
{
    m_port = port;
}

int  SerialDeviceParams::getPort()
{
    return(m_port);
}

void SerialDeviceParams::setBaudRate(int rate)
{
    m_baudRate = rate;
}

int SerialDeviceParams::getBaudRate()
{
    return(m_baudRate);
}

void SerialDeviceParams::setDataBits(SerialComm::E_DataBits bits)
{
    m_dataBits = bits;
}

SerialComm::E_DataBits SerialDeviceParams::getDataBits()
{
    return(m_dataBits);
}

void SerialDeviceParams::setStopBits(SerialComm::E_StopBits bits)
{
    m_stopBits = bits;
}

SerialComm::E_StopBits SerialDeviceParams::getStopBits()
{
    return(m_stopBits);
}

void SerialDeviceParams::setParity(SerialComm::E_Parity parity)
{
    m_parity = parity;
}

SerialComm::E_Parity SerialDeviceParams::getParity()
{
    return(m_parity);
}

void SerialDeviceParams::setReadTimeout(int readTimeout)
{
    m_readTimeout = readTimeout;
}

int SerialDeviceParams::getReadTimeout()
{
    return(m_readTimeout);
}
