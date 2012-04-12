/*****************************************************************************
 * IMU Data Console (IDC)
 *****************************************************************************
 *
 * Primary Author: Keith Benson
 * Secondary Author: Chris Konvalin
 *
 * Purpose:
 * Console application that provides basic functionality to interface with
 * MEMSense IMU's - nIMU, uIMU and uIMU with hi-G option.  Directly interacts
 * with the SiLabs USB device via the USBExpress API v1.6.  A menuing system
 * is used to configure the communication and type of device.  The architecture
 * of this application has migrated towards the OOP paradigm.  Thus, only main.cpp,
 * MenuHelper.h/.cpp and IOHelpers.h/cpp are not class-based.  The remaining
 * application elements are class-based.
 *
 * A short description of the OO architecture/organization is appropriate.  The
 * application is primarily responsible for three things: 1. reading data from
 * the IMU, 2. parsing out the data and, 3. print the data to file/console.  As
 * such, the data generally has the following characteristics: 1. Consists of a
 * standard 13-byte header used on *all* MEMSense IMU's.  2. Payload, consisting of
 * data for the X/Y/Z components of gyro, accelerometer and magnetometer, amounting
 * to 9 total sensor values, each of which is a 2-byte value in the raw packet.
 * These two-byte values (MSB/LSB), once merged, are termed 'counts'.  The counts
 * can then be converted into engineering units and units, where units is the 'unitized'
 * value of the sensor (e.g. unitized gyro -> deg/s).  3. The payload may also contain
 * optional temperature and hi-g accelerometer values, again consisting of 2-byte
 * values which must be converted into their respective counts/engineering units/
 * units.
 *
 * The header and payloads lend themselves well to an interface class that enforces
 * a standard set of methods that *must* be implemented by all concrete
 * implementations.  Two interface classes were thus defined: IIMUHeader and
 * IIMUPayload.
 *
 * Since all current MEMSense IMU's have the same header, the concrete class
 * StandardIMUHeader, which implements IIMUHeader, encapsulates and parses the header
 * information.
 *
 * All current MEMSense IMU's also expose Gyro, Accel and Magnetometer data in what
 * is generically termed the 'payload'.  However, additional payload information
 * such as temperature and hi-g accelerometer values may also be available.  Thus,
 * each of these payload elements were broken into separate payload classes -
 * IMUMAGPayload, IMUTempPayload and IMUHiGPayload - each of which implements the
 * IIMUPayload interface.  IIMUPayload requires standard payload methods such
 * as getBytes(), getCounts(), getUnits() and getEngUnits() are implemented and
 * exposed by all concrete classes.
 *
 * Finally, to represent the header and payload information as a single packet, or
 * sample, the interface IIMUSample was defined which exposes methods which are
 * common across all samples, such as getBytes(), getPayload() and getHeader().
 * Each MEMSense IMU configuration device is represented by a concrete IMUSample
 * class:
 *      1. MicroIMUSample: gyro, accel and mag (MAG)
 *      2. MicroHiGIMUSample: MAG and hi-g accel (hi-g accel is X/Y only)
 *      3. MicroTempIMUSample: MAG and X/Y/Z temp
 *      4. MicroHiGTempIMUSample: MAG, hi-g accel and temp
 *      5. NanoIMUSample: gyro, accel, mag and temp
 *
 * The IMUSample classes are also able to read data directly from the device if
 * given a communications object which implements the IComm interface.  Once
 * the data is read from the device it is automatically parsed and available
 * to the user.
 *
 * The menu and I/O functions have been split out into two files:
 * IOHelpers.cpp/h and MenuHelper.cpp/h.  As implied by their names, IOHelpers
 * contains utility functions for writing to disk and file, and MenuHelper
 * contains utility functions for handling the menu elements.
 *
 * Lastly, the class SensorDataUtils is non-instantiable and contains only
 * static data conversion methods.  These methods may be used to convert bytes
 * read from the IMU to counts, engineering units and units.  Additional methods
 * are available that simplify calculating checksums, and working with
 * temperatures (conversion equations for temperatures are slightly different
 * from those of gyro, accel and mag).
 *
 *
 * Requirements:
 * Compiling:
 *      SiUSBXp.h
 * Linking:
 *      SiUSBXp.lib
 * Execution:
 *      SiUSBXp.dll
 *
 * Usage:
 *      I2C: IDC [-?] [-o outfile] [-c n] [-t] [-v]
 *      RS422: IDC <COMx> [-?] [-o outfile] [-c n] [-t] [-v]
 *
 *      -?: displays usage
 *      <COMx>: COM port of device - only valid for RS422 (e.g. COM4)
 *      -o <outfile>: name of data output file.  Will fail if file already exists.
 *      -c <n>: write every nth sample to console
 *      -t: display 16-bit timer value
 *      -v: IDC version
 *
 *
 * Modifications:
 *
 *   8/29/05: Initial Release
 *  10/14/05: Modified to include display of optional timer byte
 *   2/20/06: Extended IDC to work with various protocols across all devices.
 *            Added in SerialComm, USBComm and IComm.
 *            Added in PROTOCOL_TYPE block for protocol selection.
 *   3/13/06: Made extensive changes to overall app structure.  Migrated
 *              towards OO design by stripping out communication and parsing
 *              to individual classes.  Also added menu options to eliminate
 *              need for user to compile with new settings.  Can now be
 *              distributed as an executable and only recompiled if user
 *              wishes to extend existing functionality.
 *   3/22/06: Modified USB connection so does not always assume device 0 is
 *              the device to connect to.  May be device 1, 2, etc. and is
 *              access via the global g_deviceIndex.  Is set via auto-config.
 *   6/20/06: Fixed a bug in the SerialComm() class where the timeouts were
 *              not being set correctly.  The end result is the app wasn't
 *              waiting long enough in some cases for data to arrive off the
 *              line and would fail, indicating couldn't find a device.
 *            Devices are now shipping with 115200 baud rate on RS422. Added
 *              a menu option to set the baud rate manually.
 *            Auto-configuration now checks to see if the device is
 *              communicating on 57600 or 115200.
 *   8/28/06: Fixed bug 27, which involved incorrectly specifying the port
 *            for COM ports 10 and above.  Fix incorporated per MSDN article:
 *            http://support.microsoft.com/default.aspx?scid=kb;%5BLN%5D;115831
 *   3/28/07: Converted getch()/kbhit() calls to wrappers for ISO compliance.
 *              Converted to IMUDataUtils library rather than do a monolithic
 *              build; easier for incremental mods.
 *
 *****************************************************************************/
#ifdef MEMS_INTERNAL_BUILD
#include "MenuHelperInternal.h"                         // Internal/Engineering Menu Hook
#endif

#include "IMUDataUtils/SDTDefaultPre.h"

#include "GlobalVars.h"                                 // application configuration
#include "MenuHelper.h"                                 // Menu processing
#include "IDCUtils.h"                                   // wrappers for getch(), kbhit()
#include "IOHelpers.h"                                  // printing to file/console
// #include "IMUDataUtils/USBComm.h"                  // Non-serial communication class (SI_Labs)
#include "IMUDataUtils/SerialComm.h"               // Serial class (COM port)
#include "IMUDataUtils/CommonUtils.h"              // common utilities such as cls()
#include "IMUDataUtils/STDIMUSample.h"               // Interface class - base to all IMU classes
#include "IMUDataUtils/UnknownIMUSample.h"         // Unknown/Generic IMU parser

#include <fstream>      // file output
#include <iostream>     // file output
#include <iomanip>      // ios:: constants
#include <string>       // argument handling
#include <algorithm>    // std::transform()
#include <ctime>        // time
#include <cstddef>      // NULL macro

/*****************************************************************************
 * NAMESPACE MEMS - BEGIN
 *****************************************************************************/
namespace mems
{

/*****************************************************************************
 * STATIC/CONSTANT VALUES
 *****************************************************************************/
#ifdef MEMS_INTERNAL_BUILD
static const std::string IDC_VERSION = "1.2.0.Internal";
#else
static const std::string IDC_VERSION = "9.2.0.0";
#endif

/*****************************************************************************
 * COMMUNICATION SPECIFIC CONSTANTS
 *****************************************************************************/
static const int USB_READ_WRITE_TIMEOUT = 500; // ms before read timeout
static const int SERIAL_READ_TIMEOUT = 500; // ms before read timeout
static const SerialComm::E_DataBits SERIAL_DATA_BITS = SerialComm::DB8; // data bits

/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

/*****************************************************************************
 * PROTOTYPES
 *****************************************************************************/

void handleArgs(int argc, char * argv[]);
std::string getArg(int index, int argc, char * argv[]);
std::string toLower(std::string & str);
void initializeGlobals();
IComm * openComm();
void closeComm(IComm * comm);
IComm * openI2C();
IComm * openRS422();
STDIMUSample * setupParser();
void teardownParser(STDIMUSample * sample);
void pollDevice(IComm * comm, STDIMUSample * parser);
void checkStatus(int status, char * msg);
void printUsage();
void printUSBUsage();
void printSerialUsage();

/*****************************************************************************
 * NAMESPACE MEMS - END
 *****************************************************************************/
} // mems


int main(int argc, char* argv[])
{
  using namespace mems;

  IComm * comm;
  STDIMUSample * parser;

#ifdef MEMS_INTERNAL_BUILD
  MenuHelperInternal * menuHook = new MenuHelperInternal();
#else
  void * menuHook = NULL;
#endif

  initializeGlobals();
  handleArgs(argc, argv);

  while (showMainMenu(static_cast<mems::IMenuHook *> (menuHook)))
  {
    cls();

    // reset counter to determine accurate sample rate
    g_consoleSampleCount = 1;

    /**
     * Open the datafile, if one was specified.
     * If not specified, data file not opened.
     **/
    openOutfile(g_ofilename, true);

    // establish communication
    comm = openComm();

    if (comm)
    {
      // setup the data parser
      parser = setupParser();

      // begin processing data from device
      pollDevice(comm, parser);

      teardownParser(parser);
      closeComm(comm);
      closeOutfile();
    }

    // get rid of last keyboard hit
    mems_getch();
  }

  delete menuHook;

  return (0);
}

namespace mems
{

STDIMUSample * setupParser()
{
  using namespace mems;

  STDIMUSample * sample = STDIMUSample::getStandardSample(g_device, g_gyroRange, g_accelRange, g_magRange, g_higRange);

  if (g_conversion == PRE_COEFF)
  {
    SDTDefaultPre * transform = new SDTDefaultPre();
    sample->setTransform(transform);
  }
  else
  {
    SDTDefault * transform = new SDTDefault();
    sample->setTransform(transform);
  }

  return (sample);
}

void teardownParser(STDIMUSample * parser)
{
  ISensorDataTransform * transform = NULL;
  transform = parser->getTransform();

  delete parser;

  delete transform;

}

void initializeGlobals()
{
  using namespace mems;

  g_printCounter = false; // true if counter printed to console/file
  g_printConsole = true; // true if should print to console
  g_printConsoleN = 20; // every n lines printed to console
  g_consoleSampleCount = 1;// counter for printing to console
  g_gyroRange = 300.0; // range for gyro
  g_accelRange = 5.0; // range for accel
  g_magRange = 1.9; // range for mag
  g_higRange = 5.0; // range for hi-g accel (uimu)
  g_comPort = "/dev/ttyUSB0"; // COM port to receive data (RS422)
  g_baudRate = 115200; // COM port baud rate (RS422)
  g_deviceIndex = 0; // USB device index (I2C)
  g_protocol = RS422; // type of protocol
  g_device = NIMU_3TEMP; // type of device
  g_idcVersion = IDC_VERSION; // version of IDC

  //Internal build: Pre coefficient conversion and output in volts:
#ifdef MEMS_INTERNAL_BUILD
  g_conversion = PRE_COEFF;
  g_dataFormat = VOLTS; // type of data format to output
  g_verifyChecksum = true;
  //External build: Post coefficient conversion and output in engineering units:
#else
  g_conversion = POST_COEFF;
  g_dataFormat = ENGINEERING_UNITS; // type of data format to output
#endif

}

void handleArgs(int argc, char * argv[])
{
  using namespace mems;

  /**
   * Expected arguments are as follows:
   *
   * <COMx>: COM port of device (e.g. COM4)
   * -o <filename>: output data to file where <filename> is the file
   *      to be either created, or truncated to zero-length.
   * -f: force write to file.  If the file already exists and -f is
   *      not specified, we will *not* overwrite the file, but
   *      instead alert the user and exit.  If -f is specified, however,
   *      we will overwrite the file.
   * -c <N>: print every Nth sample to console.
   * -v: software version
   *
   **/
  std::string key;
  std::string val;
  bool done = false;
  int i = 1;

  while (i < argc)
  {
    key = getArg(i, argc, argv);

    if (key.compare("-o") == 0)
    {
      // should be a filename associated with this option
      ++i;
      val = getArg(i, argc, argv);
      if (val.length() > 0)
        g_ofilename = val;
      else
      {
        printUsage();
        exit(1);
      }
    }
    else if (key.compare("-c") == 0)
    {
      // should be a number associated with this option
      ++i;
      val = getArg(i, argc, argv);
      if (val.length() > 0)
      {
        g_printConsole = true;
        g_printConsoleN = atoi(val.c_str());
      }
      else
      {
        printUsage();
        exit(1);
      }
    }
    else if (key.compare("-t") == 0)
    {
      g_printCounter = true;
    }
    else if (key.compare("-v") == 0)
    {
      std::cout << "\nIMU Data Console (IDC) v" << g_idcVersion << "\n";
      exit(1);
    }
    else if ((key.find_first_of("COM") != std::string::npos) || (key.find_first_of("com") != std::string::npos))
    {
      /**
       * Convert the COM port specified to upper-case, and place the
       * results into the global g_comPort.  Note that g_comPort must
       * be sized to the correct length prior to calling transform,
       * as transform does not do allocation.  Also, note that toupper
       * is a <stdlib> function.
       **/
      g_comPort = key;
      std::transform(key.begin(), key.end(), g_comPort.begin(), toupper);
    }
    else
    {
      std::cout << "Invalid option: " << key << "\n\n";
      printUsage();
      exit(1);
    }

    ++i;
  }
}

std::string getArg(int index, int argc, char * argv[])
{
  using namespace mems;

  std::string arg;

  if (index < argc)
    arg = argv[index];

  return (arg);
}

IComm * openComm()
{
  using namespace mems;

  IComm * comm = NULL;

  //	if(g_protocol == I2C)
  //		comm = openI2C();
  //	else
  comm = openRS422();

  return (comm);
}

void closeComm(IComm * comm)
{
  using namespace mems;

  int status;

  if (comm != NULL)
  {
    comm->closeDevice(status);
    delete comm;
    comm = NULL;
  }
}

//IComm * openI2C()
//{
//	using namespace mems;
//
//    int status;
//    USBComm * comm = new USBComm;
//    USBComm::SDeviceData devData;
//
//    devData.m_deviceIndex = g_deviceIndex;
//
//    if( ! comm->openDevice(&devData, status))
//    {
//        delete comm;
//        comm = NULL;
//        checkStatus(status, "Error opening device.");
//    }
//    else
//    {
//        /**
//         * NOTE: The read/write timeouts must be set to greater than 0 (default)
//         * else will return immediately whether data is read or not.  Want the
//         * call to block until data is read.
//         **/
//        comm->setTimeouts(USB_READ_WRITE_TIMEOUT, USB_READ_WRITE_TIMEOUT, status);
//    }
//
//    return(comm);
//}

IComm * openRS422()
{
  using namespace mems;

  int status;
  SerialComm * comm = new SerialComm();
  SerialComm::SDeviceData devData;

  devData.m_comPort = g_comPort;

  if (!comm->openDevice(&devData, status))
  {
    delete comm;
    comm = NULL;
    checkStatus(status, "Error opening device");
  }
  else
  {
    comm->setBaudRate(g_baudRate);
    comm->setDataBits(SERIAL_DATA_BITS);
    comm->setStopBits(SerialComm::ONE_STOP_BIT);
    comm->setParity(SerialComm::NO_PARITY);
    comm->setReadTimeout(SERIAL_READ_TIMEOUT);
  }

  return (comm);
}

void pollDevice(IComm * comm, STDIMUSample * parser)
{
  using namespace mems;

  int status;
  bool success;
  bool validChecksum;
  int sampleCount = 0;
//  const int MAX_SAMPLE_COUNT = 1000;

  time_t startTime;
  time(&startTime);
  comm->flushBuffer(status);

  while (!mems_kbhit() /* && sampleCount < MAX_SAMPLE_COUNT */)
  {
    // use the parser to read up the sample
    success = parser->readSample(comm, status);

    /**
     * Verify the checksum and number of bytes read is correct.
     * If not ignore sample.
     */
    if (success)
    {
      validChecksum = parser->isValidChecksum();

#ifdef MEMS_INTERNAL_BUILD
      if( ! g_verifyChecksum)
      validChecksum = true;
#endif

      if (validChecksum)
      {
        // print contents to screen and/or file, depending on options.
        processSample(*parser);
        ++sampleCount;
      }
      else
      {
        std::cout << "\n-- Joan Pau -- Invalid cheksum";
      }
    }
  }

  // Calculate actual sample Rate
  double timeUsed = difftime(time(NULL), startTime);
  std::cout << "\n\n";
  std::cout << sampleCount << " messages in " << timeUsed << " seconds. (" << (sampleCount) / (timeUsed) << "HZ)\n\n";
  std::cout << "Press any key to continue...";
  mems_getch();
}

void checkStatus(int status, char * msg)
{
  using namespace mems;

  if (status != 0)
  {
    std::cerr << "\t" << msg << "\n";
    std::cerr << "\tStatus: " << status << "\n";
    std::cout << "\n\tPress any key to continue...";
    mems_getch();
  }
}

} // mems
