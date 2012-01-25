/*****************************************************************************
 * IOHelpers
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author:
 *
 * Purpose:
 *      Templated helper functions to print the payloads out to console
 *      and file.  Note that only the functions listed in the header file
 *      are intended for direct use - all others are essentially private
 *      and should not be called directly.
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
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/

#include "GlobalVars.h"                         // configuration of data output
#include "IMUDataUtils/SensorDataUtils.h"	// data helper class
#include "IMUDataUtils/STDIMUSample.h"       // access to STDIMUSample interface
#include "IMUDataUtils/IIMUPayload.h"      // access to IIMUPayload interface
#include "IMUDataUtils/IIMUHeader.h"       // access to IIMUHeader interface
#include "IMUDataUtils/IMUTempPayload.h"   // access to IMUTempPayload class
#include <vector>
#include <fstream>      // file output
#include <iostream>     // file output
#include <iomanip>      // ios:: constants
#include <cstdlib>      // exit calls
namespace mems
{
std::ofstream g_ofile; // output file stream

template<typename T>
  void handleOutput(const std::vector<T> & payload, bool addCR = true);

template<typename T>
  void handleFileOutput(const std::vector<T> & payload, bool addCR = true);

template<typename T>
  void handleConsoleOutput(const std::vector<T> & payload, bool addCR = true);

template<typename T>
  void handleOutput(const std::vector<T> & data, bool addCR)
  {
    handleConsoleOutput(data, addCR);
    handleFileOutput(data, addCR);
  }

template<typename T>
  void handleConsoleOutput(const std::vector<T> & data, bool addCR)
  {
    typename std::vector<T>::const_iterator iter;

    if (g_printConsole)
    {
      if ((g_consoleSampleCount % g_printConsoleN) == 0)
      {
        // output payload data
        for (iter = data.begin(); iter != data.end(); ++iter)
        {
          if (g_dataFormat == BYTES || g_dataFormat == COUNTS)
          {
            std::cout << std::hex << std::setfill(' ') << std::left << std::setw(11) << static_cast<short> (*iter)
                << "\t";
            std::dec(std::cout);
          }

          else
            std::cout << std::setfill(' ') << std::left << std::setw(11) << *iter << "\t";
        }

        if (addCR)
        {
          std::cout << "\n";
          g_consoleSampleCount = 1;
        }
      }
      else if (addCR)
        ++g_consoleSampleCount;
    }
  }

template<typename T>
  void handleFileOutput(const std::vector<T> & data, bool addCR)
  {
    typename std::vector<T>::const_iterator iter;

    if (g_ofile.is_open())
    {
      // output payload data
      for (iter = data.begin(); iter != data.end(); ++iter)
      {
        if (g_dataFormat == BYTES || g_dataFormat == COUNTS)
        {
          g_ofile << std::hex << std::setfill(' ') << std::left << std::setw(11) << static_cast<short> (*iter) << "\t";
          std::dec(std::cout);
        }
        else
          g_ofile << std::setfill(' ') << std::left << std::setw(7) << *iter << "\t";
      }

      if (addCR)
        g_ofile << "\n";
    }
  }

void openOutfile(std::string & name, bool forceWrite)
{
  std::ifstream bogus;

  // make sure last file, if any, is closed before opening new one.
  closeOutfile();

  /**
   * Test if should open an output file.  If so, and -f was not specified,
   * try to open first as read only to see if exists.  If dne, then reopen
   * as writable, else error out to user.  If -f specified, just open
   * as normal and overwrite if already present.
   **/
  if (name.length() > 0)
  {
    if (forceWrite)
    {
      g_ofile.open(name.c_str(), std::ios_base::out | std::ios_base::trunc);
    }
    else
    {
      // check to see if file already exists by trying to open
      bogus.open(name.c_str(), std::ios_base::in);
      if (!bogus.is_open())
      {
        // file doesn't exist, so open as writable
        g_ofile.open(name.c_str(), std::ios_base::out | std::ios_base::trunc);
      }
      else
      {
        std::cerr << "Output file already exists - specify -f option to overwrite.\n";
        exit(EXIT_FAILURE);
      }
    }

    if (!g_ofile.is_open())
    {
      std::cerr << "Failed to open output file.\n";
      exit(EXIT_FAILURE);
    }
  }
}

void closeOutfile()
{
  if (g_ofile.is_open())
    g_ofile.close();
}

void processSample(STDIMUSample & sample)
{
  size_t lastIdx = sample.getPayload().size() - 1;
  std::vector<IIMUPayload *> payloads = sample.getPayload();
  std::vector<unsigned short> counter;
  E_DataFormatType tempFormat;
  bool addCR = false;
  bool isTempPayload = false;

  if (g_printCounter)
  {
    counter.push_back(sample.getHeader()->getCounter());
    handleOutput(counter, false);
  }

  if (g_printHeader)
  {
    /*
     * To print the header correctly the format type must be set to BYTES,
     * else the underlying code will attempt to output the ascii characters.
     * Thus, save off the current format, set to BYTES, dump out the data
     * then reset back to original format.
     */
    tempFormat = g_dataFormat;
    g_dataFormat = BYTES;
    handleOutput(sample.getHeader()->getBytes(), false);
    g_dataFormat = tempFormat;
  }

  for (size_t i = 0; i <= lastIdx; ++i)
  {
    /*
     * Using dynamic_cast<> to determine if this is a temperature
     * payload.  Necessary since only have access to IIMUPayload
     * pointers, returned as part of sample.getPayload(), and the
     * option to always print temp to console in Celsius requires
     * correctly identifying the temp payload. Assuming dynamic_cast
     * will not throw an exception (old method), but rather will
     * return NULL/0 upon failure.
     */
    isTempPayload = (dynamic_cast<IMUTempPayload *> (payloads[i]) != NULL);

    // append a carriage return if last payload to process
    addCR = (i == lastIdx);

    switch (g_dataFormat)
    {
      case BYTES:

        /*
         * If option specified to always print temps in Celsius to
         * console, and this is a temperature payload, then print
         * in units, otherwise just default to normal output type.
         */
        if (isTempPayload && g_consoleTempCelsius)
          handleConsoleOutput(payloads[i]->getEngUnits(), addCR);
        else
          handleConsoleOutput(payloads[i]->getBytes(), addCR);

        /*
         * always print data to file in specified format -
         * don't convert temps.
         */
        handleFileOutput(payloads[i]->getBytes(), addCR);
        break;

      case COUNTS:
        if (isTempPayload && g_consoleTempCelsius)
          handleConsoleOutput(payloads[i]->getEngUnits(), addCR);
        else
          handleConsoleOutput(payloads[i]->getCounts(), addCR);

        handleFileOutput(payloads[i]->getCounts(), addCR);
        break;

      case ENGINEERING_UNITS:
      default:
        handleConsoleOutput(payloads[i]->getEngUnits(), addCR);
        handleFileOutput(payloads[i]->getEngUnits(), addCR);
        break;

      case VOLTS:
        if (isTempPayload && g_consoleTempCelsius)
          handleConsoleOutput(payloads[i]->getEngUnits(), addCR);
        else
          handleConsoleOutput(payloads[i]->getVolts(), addCR);
        break;
    }
  }
}

} // mems
