#ifndef MEMS_IMU_COUNTER_UTILS_H
#define MEMS_IMU_COUNTER_UTILS_H

namespace mems
{

class IMUCounterUtils
{
public:
    IMUCounterUtils(double tickValSec = 2.1701e-6);    // tick value in s
    ~IMUCounterUtils(void);

    void reinitialize();
    void updateCount(unsigned short counter);
    bool isValid();
    unsigned short getPreviousCount();
    unsigned short getCurrentCount();
    unsigned short getDeltaCounts();
    double getDeltaSeconds();
    void setTickValSeconds(double tickValSec);
    double getTickValSeconds();

private:

    enum E_ReadyState
    {
        UNINITIALIZED,
        WAIT_FOR_UPDATE,
        INITIALIZED
    };

    double          m_tickValSec;  
    unsigned short  m_previous;
    unsigned short  m_current;
    E_ReadyState    m_state;
};

} // mems

#endif // MEMS_IMU_COUNTER_UTILS_H