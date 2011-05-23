/******************************************************************************
 * Class: SerialComm
 * Author: Joan Pau on original version from ckonvalin
 * Created: 11/13/05
 *
 * Purpose:
 		Establish Serial communication with device.

 *
 * $Archive::                                                                 $
 *
 * Updates:
 * 	  $Log: $
 *
 *
 * Copyright (c) 2005, MEMSense, Inc.  All rights reserved.
 *
 *****************************************************************************/
#ifndef MEMS_SERIALCOMM_H
#define MEMS_SERIALCOMM_H

#include "IComm.h"
#include <string>
#include <termios.h>    // brings in Unix serial API
// #include "windows.h"    // brings in Windows serial API

namespace mems
{

class SerialComm : public IComm
{
public:
    SerialComm(void);
    virtual ~SerialComm(void);

    enum E_StopBits
    {
        ONE_STOP_BIT,
        // ONE_5_STOP_BITS // Not handled on GNU/Linux
        TWO_STOP_BITS
    };

    enum E_Parity
    {
        EVEN_PARITY,
        MARK_PARITY,
        NO_PARITY,
        ODD_PARITY,
        SPACE_PARITY
    };

    enum E_DataBits
    {
        DB5,
        DB6,
        DB7,
        DB8
    };

    bool openDevice(void * data, int & status);
    bool closeDevice(int & status);
    bool readData(unsigned long numBytesToRead, unsigned char * buffer,
                unsigned long & numBytesRead, int & status);
    bool writeData(unsigned long numBytesToWrite, unsigned char * buffer,
                 unsigned long & numBytesWritten, int & status);
    bool flushBuffer(int & status);
    bool setBaudRate(unsigned long rate);
    bool setDataBits(E_DataBits bits);
    bool setStopBits(E_StopBits bits);
    bool setParity(E_Parity parity);
    bool setReadTimeout(short time_ms);
    struct SDeviceData
    {
        std::string m_comPort;
    };

private:
    static const int MEMS_NO_ERROR;

    bool checkStatus(int status);
    bool getAttr(termios & tc);
    bool setAttr(termios & tc);
    bool initComm();     // initialize the serial port for raw I/O

    bool m_deviceOpen;   // true if a device is already open
    int  m_devFd;        // file descriptor to current device
};

} // mems

#endif // MEMS_SERIALCOMM_H
