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
 * Modifications:
 *   8/28/06: Fixed Mantis bug 27.  Involved prepending "\\\\.\\" to each
 *      COM port specification.  Now correctly works with ports 10 and above.
 *      See the following MSDN article for additional information:
 *      http://support.microsoft.com/default.aspx?scid=kb;%5BLN%5D;115831
 *****************************************************************************/

#include "SerialComm.h"
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>

///////////////////////////////////////////////////////////////////////////////
using namespace mems;
///////////////////////////////////////////////////////////////////////////////

const int SerialComm::MEMS_NO_ERROR = 0;

SerialComm::SerialComm(void) : m_deviceOpen(false), m_devFd(-1)
{
}

SerialComm::~SerialComm(void)
{
    int status;

    closeDevice(status);
}

bool SerialComm::openDevice(void * data, int & status)
{
    bool success = true;
    SDeviceData * devData = reinterpret_cast<SDeviceData *>(data);
    std::string comPort;

    status = MEMS_NO_ERROR;

    comPort = "";
    comPort.append(devData->m_comPort);

    // see if a device is already open.  If so, close, then open new device.
    if(m_deviceOpen)
        closeDevice(status);

    /**
     * fd = open(char* name, O_RDONLY | O_NOCTTY | O_NONBLOCK ) :
     * open device name for:
     *    1 read only,
     *    2 no control terminal (keyboard does not affect communication)
     *    3 do not wait for device answer
     **/
    m_devFd = open(
        comPort.c_str(),
        O_RDONLY | O_NOCTTY | O_NDELAY );

    if(m_devFd < 0)
    {
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        m_deviceOpen = true;
        success = initComm();
    }

    return(success);
}

bool SerialComm::initComm()
{
    bool success = false;

    termios tc;

    if ( getAttr(tc) )
    {
      tc.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
      tc.c_oflag &= ~OPOST;
      tc.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
      tc.c_cflag &= ~(CRTSCTS|HUPCL);
      tc.c_cflag |= (CREAD|CLOCAL);

      success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::closeDevice(int & status)
{
    status = MEMS_NO_ERROR;

    if(m_deviceOpen)
    {
        if( close(m_devFd) < 0)
            status = errno;

        m_deviceOpen = false;
    }

    return(checkStatus(status));
}

bool SerialComm::readData
(
    unsigned long numBytesToRead,
    unsigned char * buffer,
    unsigned long & numBytesRead,
    int & status
)
{
    bool success = true;
    status = MEMS_NO_ERROR;
    long int readResult;
    readResult = read(m_devFd, buffer, numBytesToRead);
    if( readResult < 0 )
    {
        numBytesRead = 0;
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        numBytesRead = readResult;
        success = (numBytesToRead == numBytesRead);
    }

    return(success);
}

bool SerialComm::writeData
(
    unsigned long numBytesToWrite,
    unsigned char * buffer,
    unsigned long & numBytesWritten,
    int & status
)
{
    bool success;
    status = MEMS_NO_ERROR;
    long writeResult;

    writeResult = write(m_devFd, buffer, numBytesToWrite);
    if( writeResult < 0 )
    {
        numBytesWritten = 0;
        status = errno;
        success = checkStatus(status);
    }
    else
    {
        numBytesWritten = writeResult;
        success = (numBytesToWrite == numBytesWritten);
    }

    return(success);
}

bool SerialComm::flushBuffer(int & status)
{
    status = MEMS_NO_ERROR;

    if( m_deviceOpen )
    {
        if( tcflush(m_devFd, TCIOFLUSH) < 0 ) // might it be tcdrain
            status = errno;
    }

    return(checkStatus(status));
}

bool SerialComm::setBaudRate(unsigned long rate)
{
    bool success = false;

    termios tc;
    if( getAttr(tc) && ( cfsetspeed(&tc, rate) == 0 ) )
    {
        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setDataBits(E_DataBits bits)
{
    bool success = false;

    termios tc;

    if( getAttr(tc) )
    {
        tc.c_cflag &= ~CSIZE;

        switch (bits)
        {
          case DB5:
            tc.c_cflag |= CS5;
            break;

          case DB6:
            tc.c_cflag |= CS6;
            break;

          case DB7:
            tc.c_cflag |= CS7;
            break;

          case DB8:
            tc.c_cflag |= CS8;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setStopBits(E_StopBits bits)
{
    bool success = false;

    termios tc;

    if( getAttr(tc) )
    {
        switch(bits)
        {
          case ONE_STOP_BIT:
            tc.c_cflag &= ~CSTOPB;
            break;

          case TWO_STOP_BITS:
            tc.c_cflag |= CSTOPB;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setParity(E_Parity parity)
{
    bool success = false;

    termios tc;

    if( getAttr(tc) )
    {   
        /**
         * Mark/Space parity is not well documented.
         * It may be set with the CMSPAR c_cflag
         * and the other parity flags (PARENB, PARODD).
         * In some systems it might not be available 
         * or not documented in termios.h man page.
         **/

        tc.c_cflag &= ~( PARODD | CMSPAR );

        switch(parity)
        {
        case EVEN_PARITY:
            tc.c_cflag |= PARENB;
            break;

        case MARK_PARITY:
            tc.c_cflag |= PARENB | PARODD | CMSPAR;
            break;

        case NO_PARITY:
            tc.c_cflag &= ~PARENB;
            break;

        case ODD_PARITY:
            tc.c_cflag |= PARENB | PARODD;
            break;

        case SPACE_PARITY:
            tc.c_cflag |= PARENB | CMSPAR;
            break;
        }

        success = setAttr(tc);
    }

    return(success);
}

bool SerialComm::setReadTimeout(short time_ms)
{
    termios tc;
    bool success = false;

    if(getAttr(tc))
    {
        /**
         * MIN == 0; TIME > 0: TIME specifies the limit for a timer in
         * tenths of a  second.
         * The  timer  is  started when read(2) is called.
         * read(2) returns either when at least one byte of data is
         * available,  or  when the  timer  expires.
         * If the timer expires without any input becoming available,
         * read(2) returns 0.
         * See termios.h man page, non-canonical mode input.
         * c_cc[VMIN] and c_cc[VTIME].
         **/
        tc.c_cc[VMIN] = 0;
        tc.c_cc[VTIME] = time_ms/100;
        success = setAttr(tc);
    }

    return(success);
}


bool SerialComm::getAttr(termios & tc)
{
    bool success = true;
    if( tcgetattr(m_devFd, &tc) < 0 )
        success = false;

    return(success);
}

bool SerialComm::setAttr(termios & tc)
{
    bool success = true;
    if( tcsetattr(m_devFd, TCSANOW, &tc) < 0 )
        success = false;

    return(success);
}

bool SerialComm::checkStatus(int status)
{
    bool success = true;

    if(status != MEMS_NO_ERROR)
        success = false;

    return(success);
}
