#include "MenuHelperInternal.h"
#include "IDCUtils.h"                       // kbhit, getch
#include "GlobalVars.h"                     // communication/device data
#include "AutoConfig.h"                     // findDeviceSerial
#include "IMUMsgsCommon.h"
#include "IMUDataUtils/SerialComm.h"
#include "IMUDataUtils/CommonUtils.h"  // cls(), etc.
#include "IMUDataUtils/UnknownIMUSample.h"
#include <iostream>                         // cout
#include <fstream>
#include <sstream>                          // istringstream
#include <cstring>
#include <vector>
#include <cstdlib>                          // exit call

namespace mems
{
    static const unsigned short NUM_GYROS = 3;
    static const unsigned short NUM_ACCELS = 3;
    static const unsigned short NUM_MAGS = 3;
    static const unsigned short NUM_SENSORS = NUM_GYROS + NUM_ACCELS + NUM_MAGS;
    static const unsigned short NUM_OFFSET_SCALE_VALS_PER_SENSOR = 6;
    static const unsigned short NUM_SENS_VALS_PER_SENSOR = 3;
}

using namespace mems;

const int MenuHelperInternal::BAUD_RATES[NUM_BAUD_RATES] = {115200, 460800};


MenuHelperInternal::MenuHelperInternal(void)
{
    g_verifyChecksum = true;
    g_conversion = PRE_COEFF;
}

MenuHelperInternal::~MenuHelperInternal(void)
{

}

void MenuHelperInternal::displayMenuHeader(void * data)
{
    std::cout << "\tVerify Checksum: " << (g_verifyChecksum ? "TRUE\n" : "FALSE\n");
    std::cout << "\tConversion: " << conversionType2Name(g_conversion) << "\n";
}

void MenuHelperInternal::displayMenu(void * data)
{
    std::cout << "\th: Toggle checksum verification\n";
    std::cout << "\te: Download coefficients\n";
    std::cout << "\tv: Select conversion\n";
	 std::cout << "\tu: Send 'start data' message (uIMU only)\n";
}

bool MenuHelperInternal::handleMenuChoice(int c, void * data)
{
    bool ok = true; // always redisplay the root menu

    switch(c)
    {
    case 'v':
        handleConversion();
        break;

    case 'e':
        handleDownloadCoeffs();
        break;

	case 'u':
		handleStartDataMessage();
		break;

    case 'h':
        g_verifyChecksum = ! g_verifyChecksum;
        break;
    }

    return(ok);
}

void MenuHelperInternal::handleStartDataMessage()
{
	int c;

    do
    {
        showStartDataMessageMenu();
        c = ascii2int(mems_getch());

        if(c != 1)
		{
            handleStartDataMessageChoice(c);
			std::cout << "\nPress any key to continue...";
			mems_getch();
		}

    }while(c != 1);
}

void MenuHelperInternal::handleStartDataMessageChoice(int c)
{
	switch(c)
    {
    case 2:
		std::cout << "\n\n\tSpecify serial port (syntax: COMx): ";
		std::cin >> g_comPort;
        break;

    case 3:
		std::cout << "\n\n\tSpecify baud rate: ";
		std::cin >> g_baudRate;
        break;

	case 4:
		sendStartDataMessage();
		break;

	default:
        std::cout << "\n\n\tInvalid selection.\n";
        break;
	}
}

void MenuHelperInternal::sendStartDataMessage()
{
	static const int XON = 0x11;
	IComm * comm = openRS422();
	int status;
	unsigned long bytesWritten;

    // NOTE: adding additional byte for checksum
    std::vector<unsigned char> mesg_start(sizeof(mesg_header) + 1, 0);
	unsigned char checksum;

	if(comm)
	{
		// Send XON  message to board
		buildIMUMsgHeader(reinterpret_cast<mesg_header *>(&(mesg_start[0])),
                static_cast<char>(mesg_start.size()), XON);

        checksum = 0;
        for(std::size_t i = 0; i < mesg_start.size(); ++i)
            checksum += mesg_start[i];

        mesg_start.back() = checksum;

		if(comm->writeData(static_cast<unsigned long>(mesg_start.size()),
                &(mesg_start[0]), bytesWritten, status))
        {
		    std::cout << "\n\tSTART message successfully sent.\n";
            std::cout << "\tVerifying device is sending data...";

		    memset(&(mesg_start[0]), 0, mesg_start.size());
		    if(comm->readData(1, &(mesg_start[0]), bytesWritten, status))
                std::cout << "SUCCESS!\n";
            else
                std::cout << "Failed.\n";
        }
        else
            std::cout << "\n\tUnable to establish connection with device.  START message not sent.\n";

		comm->closeDevice(status);
	}
    else
        std::cout << "\tUnable to establish communications with the device.\n";

}

void MenuHelperInternal::showStartDataMessageMenu()
{
	cls();
    std::cout << "\t\t\tUIMU START DATA MESSAGE\n\n";

	std::cout << "\tCOM Port: " << g_comPort << "\n";
	std::cout << "\tBaud Rate: " << g_baudRate << "\n";
	std::cout << "\t===============================\n\n";

	std::cout << "\t1: Return to menu\n";
	std::cout << "\t2: Specify COM port\n";
	std::cout << "\t3: Specify baud rate\n";
	std::cout << "\t4: Send START message\n";

	std::cout << "\n\tChoice: ";
}

void MenuHelperInternal::handleConversion()
{
    int c;

    do
    {
        showConversionSelectMenu();
        c = ascii2int(mems_getch());

        if(c != 1)
		{
            handleConversionSelectChoice(c);
			std::cout << "\nPress any key to continue...";
			mems_getch();
		}

    }while(c != 1);
}

void MenuHelperInternal::showConversionSelectMenu()
{
    cls();
    std::cout << "\t\t\tSELECT CONVERSION\n\n";

    std::cout << "\t1. Return to Main Menu\n";
    std::cout << "\t2. Pre-Coefficient\n";
    std::cout << "\t3. Post-Coefficient\n";

	std::cout << "\n\tChoice [" << conversionType2Name(g_conversion) << "]: ";
}

void MenuHelperInternal::handleConversionSelectChoice(int c)
{
    switch(c)
    {
    case 2:
        g_conversion = PRE_COEFF;
        break;

    case 3:
        g_conversion = POST_COEFF;
        break;
    }
}

std::string MenuHelperInternal::conversionType2Name(DataConversion type)
{
    std::string name;

    switch(type)
    {
    case PRE_COEFF:
        name = "Pre-Coefficient";
        break;

    case POST_COEFF:
        name = "Post-Coefficient";
        break;
    };

    return(name);
}

void MenuHelperInternal::handleDownloadCoeffs()
{
    int c;

    do
    {
        showDownloadCoefficientMenu();
        c = ascii2int(mems_getch());

        if(c != 1)
		{
            handleDownloadCoefficientsChoice(c);
			std::cout << "\nPress any key to continue...";
			mems_getch();
		}

    }while(c != 1);
}

void MenuHelperInternal::showDownloadCoefficientMenu()
{
    cls();

    std::cout << "\t\t\tDOWNLOAD COEFFICIENTS\n\n";

    std::cout << "\t1. Return to Main Menu\n";
    std::cout << "\t2. Specify coefficient file [" << m_coeffFile << "]\n";
    std::cout << "\t3. Download Coefficients\n";

    std::cout << "\n\nChoice: ";
}

void MenuHelperInternal::handleDownloadCoefficientsChoice(int c)
{
    switch(c)
    {
    case 2:
        std::cout << "\nSpecify coefficient file: ";
        std::cin >> m_coeffFile;
        break;

    case 3:
        //handleCoefficientDownload();
        downloadCoeffs();
        break;
    }

}

void MenuHelperInternal::handleCoefficientDownload()
{
    bool success = false;

    if(readCoeffs())
    {
        if(downloadCoeffs())
            success = true;
    }
}

bool MenuHelperInternal::readCoeffs()
{
    std::ifstream ifile;
    bool success = false;

    std::cout << "\nSpecify coefficient file: ";
    std::cin >> m_coeffFile;

    ifile.open(m_coeffFile.c_str(), std::ios_base::in);

    if(ifile.is_open())
    {
        readOffsetScale(ifile);
        readSensitivities(ifile);
        downloadCoeffs();
    }
    else
    {
        std::cout << "Unable to open " << m_coeffFile << "\n";
        std::cout << "Press any key to continue...";
        mems_getch();
    }

    return(success);
}

bool MenuHelperInternal::readOffsetScale(std::ifstream & ifile)
{
    static const char * OFFSETSCALE_KEY = "OFFSETSCALE";
    std::string line;
    bool success = false;

    // seek beginning of file
    ifile.seekg(0, std::ios_base::beg);

    /*
     * iterate across each line of data
	 * NOTE: this is tested for success by calling basic_istream::operator void*(),
	 * which is defined in class basic_ios to return 'nonzero if good()'
     */
	while(std::getline(ifile, line, '\n') && ! success)
    {
        if(strcmp(OFFSETSCALE_KEY, line.c_str()) == 0)
        {
            m_offsetScale.clear();
            for(int i = 0; i < NUM_SENSORS; ++i)
                m_offsetScale.push_back(readLine(ifile));

            success = true;
        }
    }

    return(success);
}

bool MenuHelperInternal::readSensitivities(std::ifstream & ifile)
{
    static const char * GYRO_KEY = "GYRO";
    static const char * ACCEL_KEY = "ACCEL";
    std::string line;
    bool gyro_success = false;
    bool accel_success = false;

    // seek beginning of file
    ifile.seekg(0, std::ios_base::beg);

    /*
     * iterate across each line of data
	 * NOTE: this is tested for success by calling basic_istream::operator void*(),
	 * which is defined in class basic_ios to return 'nonzero if good()'
     */
    m_offsetScale.clear();
	while(std::getline(ifile, line, '\n') && ! gyro_success && ! accel_success)
    {
        if(strcmp(GYRO_KEY, line.c_str()) == 0)
        {
            for(int i = 0; i < NUM_GYROS; ++i)
            {
                /**
                 * NOTE: NUM_GYROS - 1 is correct.  If, for example, reading in the
                 * value for GX, then only reading in x-sensitivities for XY and XZ -
                 * XX is unecessary.  This holds for Y and Z axes as well.
                 */
                for(int i = 0; i < NUM_GYROS - 1; ++i)
                    m_offsetScale.push_back(readLine(ifile));

                gyro_success = true;
            }
        }
        else if(strcmp(ACCEL_KEY, line.c_str()) == 0)
        {
            for(int i = 0; i < NUM_ACCELS; ++i)
            {
                for(int i = 0; i < NUM_ACCELS - 1; ++i)
                    m_offsetScale.push_back(readLine(ifile));

                accel_success = true;
            }
        }
    }

    return(gyro_success || accel_success);
}


bool MenuHelperInternal::downloadCoeffs()
{
    bool success = false;

    cls();

    /**
     * 1) Identify port/device
     * 2) Send message to halt data
     * 3) Wait for data to stop; repeat from 2) if data continues
     * 4) Transmit coeffs
     * 5) Wait for response indicating success
     * 6) Restart data
     */
    //std::cout << "STEP 1: Identify device.\n\n";
    //devType = mems::findDeviceSerial();
    //if(devType != DT_INVALID)
    //{
        std::cout << "STEP 2: Halting data from device...";
        if(haltData())
        {
            std::cout << "acknowledge received - data halted.\n";
            success = true;
        }
    //}

    return(success);
}

bool MenuHelperInternal::haltData()
{
    IComm * comm = openRS422();
    bool success = false;
    int status;

    if(comm)
    {

        if(sendHaltMsg(comm))
        {
            /*if(getAck(comm, 0x25))
            {

            }
            else
            {
                std::cout << "Failed to receive ack from device.\nPress any key to continue...\n";
                mems_getch();
            }*/
        }
        else
        {
            std::cout << "Failed to send halt msg to device.\nPress any key to continue...\n";
            mems_getch();
        }

        comm->closeDevice(status);
    }

    return(success);
}

bool MenuHelperInternal::sendHaltMsg(IComm * comm)
{
    static const unsigned short MSG_SIZE = 39;
    unsigned char buffer[MSG_SIZE]; // spec says 39 is required in order to get a response.
                                    // must change for future implementations - seems like a hack, somewhat
    unsigned long numBytesWritten;
    int status;
    char checksum = 0;

    memset(buffer, 0, MSG_SIZE);
    buildIMUMsgHeader(reinterpret_cast<mesg_header *>(&(buffer[0])), MSG_SIZE, 10);

    for(int i = 0; i < MSG_SIZE - 1; ++i)
        checksum += buffer[i];

    buffer[MSG_SIZE - 1] = checksum;

    return(comm->writeData(MSG_SIZE, buffer, numBytesWritten, status));
}

bool MenuHelperInternal::getAck(IComm * comm, int msgId)
{
    static const unsigned short MAX_READS = 300;
    UnknownIMUSample sample(g_sampleSize);
    int status;
    unsigned short numReads = 0;
    bool success = false;
    bool readSuccess = false;

    readSuccess = sample.readSample(comm, status);
    while( ! success && numReads < MAX_READS && readSuccess == true)
    {
        if(sample.getHeader()->getMessageID() == msgId)
            success = true;
        else if(++numReads < MAX_READS)
            readSuccess = sample.readSample(comm, status);
    }

    return(success);
}

IComm * MenuHelperInternal::openRS422()
{
    int status;
    SerialComm * comm = new SerialComm();
    SerialComm::SDeviceData devData;

    devData.m_comPort = g_comPort;

    if( ! comm->openDevice(&devData, status))
    {
        delete comm;
        comm = NULL;
        checkStatus(status, "Error opening device");
    }

    comm->setBaudRate(g_baudRate);
    comm->setDataBits(SERIAL_DATA_BITS);
    comm->setStopBits(SerialComm::ONE_STOP_BIT);
    comm->setParity(SerialComm::NO_PARITY);
    comm->setReadTimeout(SERIAL_READ_TIMEOUT);

    return(comm);
}

void MenuHelperInternal::checkStatus(int status, char * msg)
{
	using namespace mems;

    if(status != 0)
    {
        std::cerr << msg << "\n";
        std::cerr << "Status: " << status << "\n";
        exit(EXIT_FAILURE);
    }
}

/**
 * Reads a single line of data (type float) from a file, and
 * parses out each space delimited item into a vector.  The file is
 * meant to be passed in repeatedly, as the internal file-pointer
 * data will remain across calls.
 *
 * @param ifile [in] File containing data to be read.
 *
 * @return vector<float> of data read from file
 *
 * @see #IU_CALLING_CONVENTION
 */
std::vector<float> MenuHelperInternal::readLine(std::istream & ifile)
{
    std::string line;
    std::vector<float> data;
    float val;

    /*
    * iterate across each line of data
    * NOTE: this is tested for success by calling basic_istream::operator void*(),
    * which is defined in class basic_ios to return 'nonzero if good()'
    */
    if(std::getline(ifile, line, '\n'))
    {
        if(line.length() > 0)
        {
            /*
            * input stream reading from a string
            * NOTE: due to a bug in the istrinstream object, it must
            * be instantiated each time around to work correctly.
            */
            std::istringstream ist;

            /*
            * Set stream to work with the current line
            */
            ist.str(line);

            /*
            * Extract each individual word from the line.
            * Note that spaces are discarded.
            */
            while(ist >> val)
            {
                data.push_back(val);
            }
        }
    }

    return(data);
}
