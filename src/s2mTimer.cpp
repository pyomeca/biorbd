#include "../include/s2mTimer.h"


s2mTimer::s2mTimer(bool startNow) :
    m_isStarted(false),
    m_isPaused(false),
    m_totalPauseTime(0.0)
{
    if (startNow)
        start();
}

void s2mTimer::start()
{
    m_start = std::clock();
    m_totalPauseTime = 0;
    m_isPaused = false;
    m_isStarted = true;
} // Start a timer

bool s2mTimer::isStarted(){
    return m_isStarted;
}

void s2mTimer::pause(){
    if (!m_isPaused){
        m_isPaused = true;
        m_pauseTime = std::clock();
    }
}

void s2mTimer::resume(){
    if (!m_isStarted)
        start();

    else if (m_isPaused){
        addPauseTime();
        m_isPaused = false;
    }
}

double s2mTimer::getLap()
{
    addPauseTime();

    if (m_isStarted)
        return getTime(m_start) - m_totalPauseTime;
    else
        return 0;
}

double s2mTimer::stop()
{
    if (m_isStarted){
        return getLap();
        m_isStarted = false;
    }
    else
        return 0;
}

void s2mTimer::addPauseTime(){
    if (m_isPaused){
        m_totalPauseTime += getTime(m_pauseTime);
        m_pauseTime = std::clock();
    }
}

double s2mTimer::getTime(const std::clock_t& timer){
    return ((double)(std::clock() - timer)) / CLOCKS_PER_SEC;
}
