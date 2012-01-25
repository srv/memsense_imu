#ifndef MEMS_IMU_MSGS_COMMON_H
#define MEMS_IMU_MSGS_COMMON_H


#if defined(WIN32)                 
#pragma pack(push, r1)              // save off current packing scheme using 'r1' identifier
#pragma pack(1)                     // use 1-char packing for structures
#endif // !defined(WIN32)

namespace mems
{

// forward declarations
struct mesg_header;
struct mesg_ack;

// functions
bool buildIMUMsgHeader(mesg_header * hdr, char msgSize, char msgCode);

// ----------------------------------------------------------------------------
// Header structure to prepend EVERY message to/from devices.
// ----------------------------------------------------------------------------
struct mesg_header
{
    unsigned char mesg_size;
    unsigned char mesg_deviceid;
    int messageCode;
    unsigned char reserved[6];
};

struct wireless_mesg_header		//explicit GAPS are for unpacked structure
{
    unsigned char sync[4];
    unsigned char mesg_size;
    unsigned char mesg_deviceid;
    unsigned char messageCode;
    unsigned short timer;
    unsigned char reserved;
    unsigned char powerStatus;
    unsigned short serialNumber;
};


// ----------------------------------------------------------------------------
//	mesg_ACK	-- reply to host message requests
// ----------------------------------------------------------------------------

struct mesg_ack
{
    struct mesg_header header;
    char ackMessageCode;
    unsigned short locked;
    unsigned short settingsDownloaded;
};

struct mesg_halt
{
    struct mesg_header header;
    char halt;
};

} // mems

#if defined(WIN32)

    // pop previously saved packing scheme off compiler stack so
    // takes effect for remainder of compilation
    #pragma pack(pop, r1)
#endif // WIN32

#endif // MEMS_IMU_MSGS_COMMON_H
