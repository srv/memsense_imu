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
 *   3/28/06: Removed Config.h.  Will now always include .cpp into this .h
 *      as currently required for templated functions.
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_IOHELPERS_H
#define MEMS_IOHELPERS_H

#include <string>

namespace mems
{
class STDIMUSample; // forward declare so don't bring in .h file

/**
 * Open output file.  If forceWrite == true, if file exists
 * will open and truncate.  If forceWrite == false, will
 * fail and exit if file already exists.
 **/
void openOutfile(std::string & name, bool forceWrite);
void closeOutfile();

/**
 * Processes the individual payloads encapsulated within
 * the sample by printing out to file and console (if
 * enabled).
 **/
void processSample(STDIMUSample & sample);

} // mems

/**
 * MSVC doesn't allow separation of
 * .h and .cpp files for templated functions - expects declarations
 * and definitions to be in .h file.
 **/
#include "IOHelpersImpl.h"


#endif /* MEMS_IOHELPERS_H */

