#ifndef MEMS_SERIAL_DEVICE_PARAMS_H
#define MEMS_SERIAL_DEVICE_PARAMS_H

#include "SerialComm.h"
#include "DeviceParams.h"

namespace mems
{

class SerialDeviceParams : public DeviceParams
{
public:
    SerialDeviceParams();
    SerialDeviceParams(int port, int baudRate, SerialComm::E_DataBits dataBits, 
        SerialComm::E_StopBits stopBits, SerialComm::E_Parity parity, int readTimeout);
    virtual ~SerialDeviceParams();

    void setPort(int port);
    int  getPort();

    void setBaudRate(int rate);
    int getBaudRate();

    void setDataBits(SerialComm::E_DataBits bits);
    SerialComm::E_DataBits getDataBits();

    void setStopBits(SerialComm::E_StopBits bits);
    SerialComm::E_StopBits getStopBits();

    void setParity(SerialComm::E_Parity parity);
    SerialComm::E_Parity getParity();

    void setReadTimeout(int readTimeout);
    int getReadTimeout();

private:
    int                         m_port;
    int                         m_baudRate;
    SerialComm::E_DataBits      m_dataBits;
    SerialComm::E_StopBits      m_stopBits;
    SerialComm::E_Parity        m_parity;
    int                         m_readTimeout;
};

} // mems

#endif // MEMS_SERIAL_DEVICE_PARAMS_H
