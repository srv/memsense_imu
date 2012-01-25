/*****************************************************************************
 * AutoConfig
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Attempts to automatically determine the correct configuration of
 *          the application.  Rolls across the COM/USB ports to find
 *          IMU data and, if found, uses the size of message to determine
 *          type of IMU.
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
 *   3/14/06: Initial Release
 *   6/06/06: Added in check for devices running RS422 at 57600 and 115200
 *              baud rates.  Can also be set manually by user via menu.
 *            Flushing the serial buffer before trying to synch up on 0xFF
 *              bytes.  Without flushing the buffer, although transmit rate
 *              is 115200, may see 'old' data at 57600, thus incorrectly
 *              identifying the baud rate and failing to configure/connect
 *              to device from main menu.
 *   8/28/06: bug 28: Allow user to change specify min/max when searching
 *              COM ports in auto-config.  User now allowed to specify
 *              via menu option, and accessed via global variables g_COMStart
 *              and g_COMStop.
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/

#include "AutoConfig.h"
#include "GlobalVars.h"
#include "IDCUtils.h"
#include "IMUDataUtils/SerialComm.h"
// #include "IMUDataUtils/USBComm.h"
#include "IMUDataUtils/CommonUtils.h"
#include "IMUDataUtils/StandardIMUHeader.h"
#include "IMUDataUtils/Types.h"
#include "IMUDataUtils/SerialDeviceParams.h"
// #include "IMUDataUtils/USBDeviceParams.h"

#include <iostream>
#include <string>
#include <sstream>  // ostringstream
namespace mems
{

/**
 * NOTE: By using NUM_BAUD_RATES to specify the number of elements in BAUD_RATES,
 * if a later change adds an additional rate and fails to increment the number
 * of elements in the array, the compiler should flag that the number of
 * elements is greater than what was allocatd (i.e. NUM_BAUD_RATES !=
 * number of elements in BAUD_RATES array).
 */
static const short NUM_BAUD_RATES = 2;
static const int BAUD_RATES[NUM_BAUD_RATES] = {115200, 460800};
static const int SERIAL_DATA_BITS = 8; // data bits
static const int USB_READ_WRITE_TIMEOUT = 500; // ms before read timeout
static const int SERIAL_READ_TIMEOUT = 500; // ms before read timeout
static const short MAX_USB_DEVICE_ID = 10;

bool autoConfig()
{
  bool success = false;

  cls();
  std::cout << "Starting auto configuration.....\n";

  g_device = DT_INVALID;
  success = findDeviceSerial();
  //    if( ! success)
  //        success = findDeviceUSB();

  if (g_device != DT_INVALID)
  {
    success = true;
    std::cout << "\nSuccessfully identified device: " << deviceType2Name(g_device);

    /**
     * UIMU_TEMP and NIMU have the same packet sizes, so no easy way of
     * identifying which is which.  So, just indicate could be either.
     */
    if (g_device == mems::UIMU_TEMP)
      std::cout << " or " << deviceType2Name(NIMU) << "\n";
    else
      std::cout << "\n";

    std::cout << "Successfully identified protocol: " << protocolType2Name(g_protocol) << "\n";
  }
  else
    std::cout << "\nUnable to identify device and/or protocol.\nManually configure device.";

  return (success);
}

bool findDeviceSerial()
{
  using namespace mems;

  SerialDeviceParams params;
  unsigned short baudRateIdx;
  bool success = false;
  short port;
  //    static const unsigned short MAX_COM_PORT = 10;

  std::cout << "\nSearching for devices using RS-422 (hit any key to return to Main Menu)...\n";

  // set basic serial port params
  params.setDataBits(SerialComm::DB8);
  params.setStopBits(SerialComm::ONE_STOP_BIT);
  params.setParity(SerialComm::NO_PARITY);
  params.setReadTimeout(500);
  params.setNumSynchBytes(StandardIMUHeader::NUM_SYNCH_BYTES);
  params.setSynchByte(StandardIMUHeader::SYNCH_BYTE);

  // start COM port search at port 3
  port = g_COMStart;
  while (port <= g_COMStop && !success && !mems_kbhit())
  {
    params.setPort(port);

    for (baudRateIdx = 0; baudRateIdx < NUM_BAUD_RATES && !success && !mems_kbhit(); ++baudRateIdx)
    {
      params.setBaudRate(BAUD_RATES[baudRateIdx]);

      std::cout << "\tAttempting to open ttyUSB" << port << " @ " << BAUD_RATES[baudRateIdx] << " bps......";

      if (mems::findDeviceSerial(params))
      {
        g_device = params.getDeviceType();
        g_protocol = params.getProtocol();

        std::ostringstream ost;
        ost << "/dev/ttyUSB" << port;
        g_comPort = ost.str();

        g_sampleSize = params.getSampleSize();
        g_baudRate = BAUD_RATES[baudRateIdx];

        success = true;

        std::cout << "SUCCESS!\n";

      }
      else
      {
        std::cout << "failed\n";
      }
    }

    ++port;
  }

  return (success);
}

//bool findDeviceUSB()
//{
//    USBDeviceParams params;
//    bool success = false;
//    short device;
//
//    std::cout << "Searching USB identifiers...\n";
//
//    // start USB device search at 0
//    device = 0;
//    while(device <= MAX_USB_DEVICE_ID && ! success && ! mems_kbhit())
//    {
//        params.setDeviceIndex(device);
//		params.setNumSynchBytes(StandardIMUHeader::NUM_SYNCH_BYTES);
//		params.setSynchByte(StandardIMUHeader::SYNCH_BYTE);
//
//        std::cout << "\tAttempting to open device " << device << "......";
//        if(mems::findDeviceUSB(params))
//        {
//            g_device = params.getDeviceType();
//            g_protocol = params.getProtocol();
//            g_sampleSize = params.getSampleSize();
//            g_deviceIndex = params.getDeviceIndex();
//
//            success = true;
//
//            std::cout << "SUCCESS!\n";
//        }
//        else
//            std::cout << "failed\n";
//
//        ++device;
//    }
//
//    return(success);
//}

}
