/*****************************************************************************
 * IDCUtils
 *****************************************************************************
 *
 * Primary Author: Chris Konvalin
 * Secondary Author: 
 *
 * Purpose: 
 *      Globally accessible utility functions.
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
 *   3/28/07: Initial Release
 *
 * Copyright(c) 2007 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_IDC_UTILS_H
#define MEMS_IDC_UTILS_H

namespace mems
{

/**
 * replace the system versions with functions that
 * appropriately call either the ISO compliant 
 * functions - if available - or revert to the 
 * non-compliant versions.
 */
int mems_getch();
int mems_kbhit();

/**
 * Moved here from IMUDataUtils/CommonUtils by Joan Pau
 */
void cls();
int ascii2int(int c);

} // mems

#endif // MEMS_IDC_UTILS_H
