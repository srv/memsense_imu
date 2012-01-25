#include "IMUMsgsCommon.h"
#include <memory.h>

using namespace mems;

bool mems::buildIMUMsgHeader(mesg_header * hdr, char msgSize, char msgCode)
{
    memset(hdr, 0, sizeof(mesg_header));
    //memset(&(hdr->sync[0]), 0xFF, 4);
    hdr->mesg_size = msgSize;
    hdr->messageCode = msgCode;

    return(true);
}

