/*****************************************************************************
 * MenuHelper
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Displays and handles all menu-related options for this application.
 *      Called from main.cpp.
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
 *   3/14/06: Moved several functions to CommonUtils.h/cpp
 *            Moved functions the user won't use (everything but showMainMenu)
 *                  into the .cpp file - better insulation.
 *   6/06/06: Added in option to manually set baud rate.
 *   8/28/06: bug 28: Allow user to change specify min/max when searching
 *              COM ports in auto-config.  User now allowed to specify
 *              via menu option 's', and sets global variables g_COMStart
 *              and g_COMStop.
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#include "MenuHelper.h"
#include "AutoConfig.h"
#include "GlobalVars.h"
#include "IDCUtils.h"
#include "IMenuHook.h"

#include "IMUDataUtils/CommonUtils.h"

#include <string>
#include <iostream>

namespace mems
{

bool handleMainMenuChoice(int c);
void handleDeviceSelect();
void showDeviceSelectMenu();
void handleDeviceSelectChoice(int c);
void handleProtocolSelect();
void showProtocolSelectMenu();
void handleProtocolSelectChoice(int c);
void handleCOMPortSelect();
void handleCOMPortSearchRange();
void showCOMSearchRangeMenu();
void showCOMSearchRangeChoice(int c);
void handleConfigurationOptions();
void showConfigurationMenu();
void handleConfigurationMenuChoice(int c);
void printUsage();
void printUSBUsage();
void printSerialUsage();
void handleRangeChoice(int c);
void showRangeMenu();
void handleRangeSpecify();
void handleBaudRateSelect();
}

bool mems::showMainMenu(IMenuHook * menuHook, void * hookData)
{
  bool success = false;
  bool hookOK = true;
  int c;

  do
  {
    cls();
    std::cout << "IMU Data Console (IDC) v" << g_idcVersion << "\n\n";
    std::cout << "\t\t\tMAIN MENU\n\n";
    std::cout << "\tDevice: " << deviceType2Name(g_device);

    /**
     * UIMU_TEMP and NIMU have the same packet sizes, so no easy way of
     * identifying which is which.  So, just indicate could be either.
     */
    if (g_device == mems::UIMU_TEMP)
      std::cout << " or " << deviceType2Name(NIMU) << "\n";
    else
      std::cout << "\n";

    std::cout << "\tProtocol: " << protocolType2Name(g_protocol) << "\n";

    if (g_protocol == RS422)
    {
      std::cout << "\tPort: " << g_comPort << "\n";
      std::cout << "\tBaud Rate: " << g_baudRate << "\n";
    }

    std::cout << "\tFormat: " << dataFormatType2Name(g_dataFormat) << "\n";
    std::cout << "\tPrint Counter: " << (g_printCounter ? "TRUE\n" : "FALSE\n");
    std::cout << "\tPrint Header: " << (g_printHeader ? "TRUE\n" : "FALSE\n");
    std::cout << "\tConsole Temp in Celsius: " << (g_consoleTempCelsius ? "TRUE\n" : "FALSE\n");
    std::cout << "\tGyro: " << g_gyroRange << " deg/s\n";
    std::cout << "\tAccel: " << g_accelRange << " G\n";
    std::cout << "\tMag: " << g_magRange << " Gauss\n";

    if (g_device == UIMU_HIG || g_device == UIMU_HIG_TEMP)
      std::cout << "\tHiG Accel: " << g_higRange << " G\n";

    if (menuHook)
      menuHook->displayMenuHeader(hookData);

    std::cout << "\t===============================\n\n";

    std::cout << "\ta: Auto-configure communications\n";
    std::cout << "\tc: Connect to device\n";
    std::cout << "\td: Select device\n";
    std::cout << "\tp: Select protocol\n";

    if (g_protocol == RS422)
    {
      std::cout << "\tm: Specify COM port\n";
      std::cout << "\tb: Specify baud rate\n";
    }

    std::cout << "\ts: COM port search range\n";
    std::cout << "\tr: Specify sensor ranges\n";
    std::cout << "\to: Configuration Options\n";
    std::cout << "\tf: Refresh the screen\n";

    // display user-defined menu, if available
    if (menuHook)
    {
      menuHook->displayMenu(hookData);
    }

    std::cout << "\tx: Exit\n";

    std::cout << "\n\t Choice: ";

    c = mems_getch();

    std::cout << "\n";

    if (c != 'c' && c != 'x')
    {
      if (!handleMainMenuChoice(c))
      {
        if (menuHook)
          hookOK = menuHook->handleMenuChoice(c, hookData);
        else
        {
          std::cout << "Invalid selection.  Press any key to continue...";
          mems_getch();
        }
      }
    }

  } while (c != 'c' && c != 'x' && hookOK);

  if (c == 'c')
    success = true;

  return (success);
}

bool mems::handleMainMenuChoice(int c)
{
  bool handled = true;

  switch (c)
  {
    case 'a':
      autoConfig();
      std::cout << "Press any key to continue...\n";
      mems_getch();
      break;

    case 'b':
      handleBaudRateSelect();
      break;

    case 'd':
      handleDeviceSelect();
      break;

    case 'p':
      handleProtocolSelect();
      break;

    case 'm':
      handleCOMPortSelect();
      break;

    case 's':
      handleCOMPortSearchRange();
      break;

    case 'r':
      handleRangeSpecify();
      break;

    case 'o':
      handleConfigurationOptions();
      break;

    case 'f':
      // refreshing, so do nothing
      break;

    default:
      handled = false;
      break;
  }

  return (handled);
}

void mems::handleDeviceSelect()
{
  int c;

  do
  {
    showDeviceSelectMenu();
    c = ascii2int(mems_getch());

    if (c != 1)
    {
      handleDeviceSelectChoice(c);
      std::cout << "\nPress any key to continue...";
      mems_getch();
    }

  } while (c != 1);
}

void mems::showDeviceSelectMenu()
{
  cls();
  std::cout << "\t\t\tSELECT DEVICE\n\n";

  std::cout << "\t1. Return to Main Menu\n";
  std::cout << "\t2. uIMU\n";
  std::cout << "\t3. uIMU w/Hi-G Acceleromter\n";
  std::cout << "\t4. uIMU w/Temp\n";
  std::cout << "\t5. uIMU w/Temp and Hi-G Accelerometer\n";
  std::cout << "\t6. nIMU\n";
  std::cout << "\t7. Uknown/Generic IMU\n";

  std::cout << "\n\tChoice [" << deviceType2Name(g_device) << "]: ";
}

void mems::handleBaudRateSelect()
{
  std::cout << "\nSpecify baud rate: ";
  std::cin >> g_baudRate;
}

void mems::handleDeviceSelectChoice(int c)
{
  int numTemp;
  int numHiG;
  switch (c)
  {
    case 2:
      g_device = UIMU;
      break;

    case 3:
      g_device = UIMU_HIG;

      //Get the number of HiG Sensors:
      do
      {
        std::cout << "\n\tNumber of Hi-G outputs (2 or 3) [Default 3]: ";
        std::cin >> numHiG;
      } while (numHiG != 2 && numHiG != 3);

      //If we have 3 axes, change the device type, otherwise leave it as the
      //standard UIMU_HIG:
      if (numHiG == 3)
        g_device = UIMU_HIG_3AXES;
      break;

    case 4:
      g_device = UIMU_TEMP;

      //Collect the number of temperature sensors in the device, then use that
      //to either change or not change the device type:
      do
      {
        std::cout << "\n\tNumber of temperature outputs (1 or 3) [Default 1]: ";
        std::cin >> numTemp;
      } while (numTemp != 1 && numTemp != 3);

      //If we have 3 axes, change the device type:
      if (numTemp == 3)
        g_device = UIMU_TEMP_3AXES;

      break;

    case 5:
      g_device = UIMU_HIG_TEMP;
      //Get the number of HiG Sensors:
      do
      {
        std::cout << "\n\tNumber of Hi-G outputs (2 or 3) [Default 3]: ";
        std::cin >> numHiG;
      } while (numHiG != 2 && numHiG != 3);

      //Collect the number of temperature sensors in the device:
      do
      {
        std::cout << "\n\tNumber of temperature outputs (1 or 3) [Default 1]: ";
        std::cin >> numTemp;
      } while (numTemp != 1 && numTemp != 3);

      //Now choose the device type based on the number of temperatures
      //and hiG sensors:
      if (numHiG == 2 && numTemp == 1)
        g_device = UIMU_HIG_TEMP;
      else if (numHiG == 2 && numTemp == 3)
        g_device = UIMU_HIG_TEMP_3AXES;
      else if (numHiG == 3 && numTemp == 1)
        g_device = UIMU_HIG_3AXES_TEMP;
      else
        //3 hiG, 3 temp
        g_device = UIMU_HIG_3AXES_TEMP_3AXES;

      break;

    case 6:
      g_device = NIMU;

      //Collect the number of temperature sensors in the device:
      do
      {
        std::cout << "\n\tNumber of temperature outputs (1 or 3) [Default 1]: ";
        std::cin >> numTemp;
      } while (numTemp != 1 && numTemp != 3);

      //3 temperatres is a different device type:
      if (numTemp == 3)
        g_device = NIMU_3TEMP;

      break;

    case 7:
      g_device = UNKNOWN_IMU;
      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }
}

void mems::handleProtocolSelect()
{
  int c;

  do
  {
    showProtocolSelectMenu();
    c = ascii2int(mems_getch());

    if (c != 1)
    {
      handleProtocolSelectChoice(c);
      std::cout << "\nPress any key to continue...";
      mems_getch();
    }

  } while (c != 1);
}

void mems::showProtocolSelectMenu()
{
  cls();
  std::cout << "\t\t\tSELECT PROTOCOL\n\n";

  std::cout << "\t1. Return to Main Menu\n";
  std::cout << "\t2. RS-422\n";
  std::cout << "\t3. I2C   (this option is disabled, Joan Pau) \n";  // Commented by Joan Pau

  std::cout << "\n\tChoice [" << protocolType2Name(g_protocol) << "]: ";
}

void mems::handleProtocolSelectChoice(int c)
{
  switch (c)
  {
    case 2:
      g_protocol = RS422;
      break;

//    case 3:
//      g_protocol = I2C;
//      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }
}

void mems::handleCOMPortSelect()
{
  std::cout << "\nSpecify serial port (syntax: COMx): ";
  std::cin >> g_comPort;
}

void mems::handleCOMPortSearchRange()
{
  int c;

  do
  {
    showCOMSearchRangeMenu();
    c = ascii2int(mems_getch());

    if (c != 1)
    {
      showCOMSearchRangeChoice(c);
      std::cout << "\nPress any key to continue...";
      mems_getch();
    }

  } while (c != 1);
}

void mems::showCOMSearchRangeMenu()
{
  cls();

  std::cout << "\t\t\tCOM Port Search Range\n\n";
  std::cout << "\t1. Back to Main Menu\n";
  std::cout << "\t2. Specify start port[" << g_COMStart << "]\n";
  std::cout << "\t3. Specify end port[" << g_COMStop << "]\n";

  std::cout << "\n\tChoice: ";
}

void mems::showCOMSearchRangeChoice(int c)
{
  int port;

  switch (c)
  {
    case 2:
      std::cout << "Starting port: ";
      std::cin >> port;

      if (port > g_COMStop)
        std::cout << "\n\nError: Starting COM port must be " << "less than ending COM port.\n";
      else
        g_COMStart = port;

      break;

    case 3:
      std::cout << "Ending port: ";
      std::cin >> port;

      if (port < g_COMStart)
        std::cout << "\n\nError: Ending COM port must be " << "greater than starting COM port.\n";
      else
        g_COMStop = port;
      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }
}

void mems::handleRangeSpecify()
{
  int c;

  do
  {
    showRangeMenu();
    c = ascii2int(mems_getch());

    if (c != 1)
    {
      handleRangeChoice(c);
      std::cout << "\nPress any key to continue...";
      mems_getch();
    }

  } while (c != 1);
}

void mems::showRangeMenu()
{
  cls();

  std::cout << "\t\t\tCONFIGURATION\n\n";
  std::cout << "\t1. Back to Main Menu\n";
  std::cout << "\t2. Specify gyro range[" << g_gyroRange << "]\n";
  std::cout << "\t3. Specify accel range[" << g_accelRange << "]\n";
  std::cout << "\t4. Specify mag range[" << g_magRange << "]\n";

  if (g_device == UIMU_HIG || g_device == UIMU_HIG_TEMP)
    std::cout << "\t5. Specify hi-g accel range[" << g_higRange << "]\n";

  std::cout << "\n\tChoice: ";
}

void mems::handleRangeChoice(int c)
{
  switch (c)
  {
    case 2:
      std::cout << "Gyro range: ";
      std::cin >> g_gyroRange;
      break;

    case 3:
      std::cout << "Accel range: ";
      std::cin >> g_accelRange;
      break;

    case 4:
      std::cout << "Mag range: ";
      std::cin >> g_magRange;
      break;

    case 5:
      std::cout << "Hi-g accel range: ";
      std::cin >> g_higRange;
      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }
}

void mems::handleConfigurationOptions()
{
  int c;

  do
  {
    showConfigurationMenu();
    c = mems_getch();

    if (c != 'x')
    {
      handleConfigurationMenuChoice(c);
      std::cout << "\nPress any key to continue...";
      mems_getch();
    }

  } while (c != 'x');
}

void mems::showConfigurationMenu()
{
  cls();

  std::cout << "\t\t\tCONFIGURATION\n\n";

  std::cout << "\tx. Back to Main Menu.\n";
  std::cout << "\to. Toggle console data [" << (g_printConsole ? "ENABLED" : "DISABLED") << "]\n";
  std::cout << "\tn. Toggle counter [" << (g_printCounter ? "ENABLED" : "DISABLED") << "]\n";
  std::cout << "\th. Toggle header data [" << (g_printHeader ? "ENABLED" : "DISABLED") << "]\n";
  std::cout << "\tt. Toggle display console temp in Celsius [" << (g_consoleTempCelsius ? "ENABLED" : "DISABLED")
      << "]\n";
  std::cout << "\ti. Set console data interval [" << g_printConsoleN << "]\n";
  std::cout << "\tb. Data output in bytes.\n";
  std::cout << "\tc. Data output in counts.\n";
  if (g_conversion == POST_COEFF)
    std::cout << "\te. Data output in engineering units.\n";
  else
    std::cout << "\tv. Data output in volts.\n";

  std::cout << "\tf. Enter output file name [" << g_ofilename << "]\n";
  std::cout << "\n\tChoice: ";
}

void mems::handleConfigurationMenuChoice(int c)
{
  using namespace mems;

  switch (c)
  {
    case 'x':
      // exit back to Main Menu - do nothing
      break;

    case 'o':
      g_printConsole = !g_printConsole;
      std::cout << "\n\n\tConsole data " << (g_printConsole ? "enabled" : "disabled") << ".\n";
      break;

    case 'h':
      g_printHeader = !g_printHeader;
      std::cout << "\n\nHeader data " << (g_printHeader ? "enabled" : "disabled") << ".\n";
      break;

    case 't':
      g_consoleTempCelsius = !g_consoleTempCelsius;
      std::cout << "\n\nPrint Temp in Celsius " << (g_consoleTempCelsius ? "enabled" : "disabled") << ".\n";
      break;

    case 'i':
      std::cout << "\n\n\tSample interval: ";
      std::cin >> g_printConsoleN;
      break;

    case 'b':
      std::cout << "\n\n\tData set to bytes.";
      g_dataFormat = BYTES;
      break;

    case 'c':
      std::cout << "\n\n\tData set to counts.";
      g_dataFormat = COUNTS;
      break;

    case 'e':
      if (g_conversion == POST_COEFF)
      {
        std::cout << "\n\n\tData output in engineering units.\n";
        g_dataFormat = ENGINEERING_UNITS;
      }
      else
      {
        std::cout << "\n\n\tInvalid selection.\n";
      }
      break;

    case 'v':
      if (g_conversion == PRE_COEFF)
      {
        std::cout << "\n\n\tData output in volts.\n";
        g_dataFormat = VOLTS;
      }
      else
      {
        std::cout << "\n\n\tInvalid selection.\n";
      }
      break;

    case 'f':
      std::cout << "\n\n\tEnter filename (enter 'x' to exit or stop printing to file): ";
      std::cin >> g_ofilename;

      if (g_ofilename.compare("x") == 0)
        g_ofilename.clear();

      break;

    case 'n':
      g_printCounter = !g_printCounter;
      break;

    default:
      std::cout << "\n\n\tInvalid selection.\n";
      break;
  }
}

void mems::printUsage()
{
  if (g_protocol == I2C)
    printUSBUsage();
  else
    printSerialUsage();
}

void mems::printUSBUsage()
{
  std::cout << "Usage: MEMS_IDC [-?] [-o outfile] [-c n] [-f]\n\n";
  std::cout << "\t-?: Display this message.\n";
  std::cout << "\t-o <outfile>: Write data to file <outfile>.\n";
  std::cout << "\t-c <n>: Display every nth data sample to console.\n";
  std::cout << "\t-t: Display sequential counter values.\n\n";

  std::cout << "Example: Write data to 'outfile.txt', print every sample to console.\n\n";
  std::cout << "MEMS_IDC -o outfile.txt -c 1\n";

}

void mems::printSerialUsage()
{
  std::cout << "Usage: MEMS_IDC <COMx> [-?] [-o outfile] [-c n] [-f]\n\n";
  std::cout << "COMx: COM port of device (e.g. COM4).\n";
  std::cout << "\t-?: Display this message.\n";
  std::cout << "\t-o <outfile>: Write data to file <outfile>.\n";
  std::cout << "\t-c <n>: Display every nth data sample to console.\n";
  std::cout << "\t-t: Display sequential counter values.\n";

  std::cout << "Example: Receive data on COM4, write data to 'outfile.txt', print every sample to console.\n\n";
  std::cout << "MEMS_IDC COM4 -o outfile.txt -c 1\n";
}

