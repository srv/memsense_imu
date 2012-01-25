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
 *
 * Copyright(c) 2006 - MEMSense, LLC USA
 *****************************************************************************/
#ifndef MEMS_MENU_HELPER_H
#define MEMS_MENU_HELPER_H

#include <string>

namespace mems
{
// forward declard hook class
class IMenuHook;

bool showMainMenu(IMenuHook * menuHook = NULL, void * hookData = NULL);

} // mems

#endif // MEMS_MENU_HELPER_H