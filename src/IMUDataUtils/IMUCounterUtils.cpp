#include "IMUCounterUtils.h"
#include <limits.h>             // USHRT_MAX

using namespace mems;


IMUCounterUtils::IMUCounterUtils(double tickValSec)
{
    setTickValSeconds(tickValSec);
    reinitialize();
}

IMUCounterUtils::~IMUCounterUtils(void)
{

}

void IMUCounterUtils::reinitialize()
{
    m_state = UNINITIALIZED;
    m_previous = 0;
    m_current = 0;
}


void IMUCounterUtils::updateCount(unsigned short counter)
{
    m_previous = m_current;
    m_current = counter;

    switch(m_state)
    {
    case UNINITIALIZED:
        m_state = WAIT_FOR_UPDATE;
        break;

    case WAIT_FOR_UPDATE:
        m_state = INITIALIZED;
        break;
    
    default:
        break;
    }
}

unsigned short IMUCounterUtils::getPreviousCount()
{
    return(m_previous);
}

unsigned short IMUCounterUtils::getCurrentCount()
{
    return(m_current);
}

unsigned short IMUCounterUtils::getDeltaCounts()
{
    unsigned short delta;

    if(m_state == INITIALIZED)
    {
        if(m_current > m_previous)
        {
            delta = m_current - m_previous;
        }
        else if(m_previous > m_current)
        {
            delta = USHRT_MAX - m_previous + m_current;
        }
        else if(m_current == m_previous)
        {
            delta = 0;
        }
    }
    else
    {
        delta = 0;
    }

    return(delta);
}

double IMUCounterUtils::getDeltaSeconds()
{
    return(getDeltaCounts() * m_tickValSec);
}

bool IMUCounterUtils::isValid()
{
    return(m_state == INITIALIZED);
}

void IMUCounterUtils::setTickValSeconds(double tickValSec)
{
    m_tickValSec = tickValSec;
}

double IMUCounterUtils::getTickValSeconds()
{
    return(m_tickValSec);
}

