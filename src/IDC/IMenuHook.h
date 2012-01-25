#ifndef MEMS_IMENU_HOOK_H
#define MEMS_IMENU_HOOK_H

#include <cstddef> // NULL macro

namespace mems
{

class IMenuHook
{
public:
    IMenuHook(){};
    virtual ~IMenuHook(){};

    virtual void displayMenuHeader(void * data = NULL) = 0;
    virtual void displayMenu(void * data = NULL) = 0;
    virtual bool handleMenuChoice(int c, void * data = NULL) = 0;
};

}


#endif // MEMS_IMENU_HOOK_H
