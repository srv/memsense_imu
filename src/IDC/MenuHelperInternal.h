#ifndef MEMS_MENU_HELPER_INTERNAL_H
#define MEMS_MENU_HELPER_INTERNAL_H

#include "IMenuHook.h"
#include "GlobalVarsInternal.h" // global vars unique to internal IDC
#include "GlobalVars.h"
#include "IMUDataUtils/SerialComm.h"
#include <string>
#include <vector>
#include <fstream>
#include <cstddef> // NULL macro

namespace mems
{
// forward declarations
class IComm;

class MenuHelperInternal : public IMenuHook
{
public:

    MenuHelperInternal(void);
    virtual ~MenuHelperInternal(void);

    virtual void displayMenu(void * data = NULL);
    virtual bool handleMenuChoice(int c, void * data = NULL);
    virtual void displayMenuHeader(void * data = NULL);

private:


    enum SensIndex
    {
        XY = 0,
        XZ,
        YX,
        YZ,
        ZX,
        ZZ
    };

    enum OffsetScaleIndex
    {
        GX = 0,
        GY,
        GZ,
        AX,
        AY,
        AZ,
        MX,
        MY,
        MZ
    };

    void handleConversion();
    void handleDownloadCoeffs();
    void showConversionSelectMenu();
    void handleConversionSelectChoice(int c);
    void showDownloadCoefficientMenu();
    void handleDownloadCoefficientsChoice(int c);
    std::string conversionType2Name(DataConversion type);
    void handleCoefficientDownload();
    bool readCoeffs();
    bool downloadCoeffs();
    bool readOffsetScale(std::ifstream & ifile);
    bool readSensitivities(std::ifstream & ifile);
    std::vector<float> readLine(std::istream & ifile);
    bool haltData();
    IComm * openRS422();
    void checkStatus(int status, char * msg);
    bool sendHaltMsg(IComm * comm);
    bool getAck(IComm * comm, int msgId);

	void handleStartDataMessage();
	void handleStartDataMessageChoice(int c);
	void showStartDataMessageMenu();
	void sendStartDataMessage();

    /**
     * NOTE: By using NUM_BAUD_RATES to specify the number of elements in BAUD_RATES,
     * if a later change adds an additional rate and fails to increment the number
     * of elements in the array, the compiler should flag that the number of
     * elements is greater than what was allocatd (i.e. NUM_BAUD_RATES !=
     * number of elements in BAUD_RATES array).
     */
    static const short  NUM_BAUD_RATES = 2;
    static const int    BAUD_RATES[NUM_BAUD_RATES];
    static const SerialComm::E_DataBits    SERIAL_DATA_BITS = SerialComm::DB8;          // data bits
    static const int    USB_READ_WRITE_TIMEOUT = 500;  // ms before read timeout
    static const int    SERIAL_READ_TIMEOUT = 500;     // ms before read timeout

    std::string         m_coeffFile;
    std::vector<std::vector<float> > m_offsetScale;
    std::vector<std::vector<float> > m_gyroSens;
    std::vector<std::vector<float> > m_accelSens;
};

} // mems

#endif // MEMS_MENU_HELPER_INTERNAL_H
