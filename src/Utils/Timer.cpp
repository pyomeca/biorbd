#define BIORBD_API_EXPORTS
#include "Utils/Timer.h"

biorbd::utils::Timer::Timer(bool startNow) :
    m_isStarted(std::make_shared<bool>(false)),
    m_isPaused(std::make_shared<bool>(false)),
    m_start(std::make_shared<std::clock_t>()),
    m_pauseTime(std::make_shared<std::clock_t>()),
    m_totalPauseTime(std::make_shared<double>(0.0))
{
    if (startNow)
        start();
}

biorbd::utils::Timer biorbd::utils::Timer::DeepCopy() const
{
    biorbd::utils::Timer copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Timer::DeepCopy(const biorbd::utils::Timer &other)
{
    *m_isStarted = *other.m_isStarted;
    *m_isPaused = *other.m_isPaused;
    *m_start = *other.m_start;
    *m_pauseTime = *other.m_pauseTime;
    *m_totalPauseTime = *other.m_totalPauseTime;
}

void biorbd::utils::Timer::start()
{
    *m_start = std::clock();
    *m_totalPauseTime = 0;
    *m_isPaused = false;
    *m_isStarted = true;
} // Start a timer

bool biorbd::utils::Timer::isStarted(){
    return *m_isStarted;
}

void biorbd::utils::Timer::pause(){
    if (!*m_isPaused){
        *m_isPaused = true;
        *m_pauseTime = std::clock();
    }
}

void biorbd::utils::Timer::resume(){
    if (!*m_isStarted)
        start();

    else if (*m_isPaused){
        addPauseTime();
        *m_isPaused = false;
    }
}

double biorbd::utils::Timer::getLap()
{
    addPauseTime();

    if (*m_isStarted)
        return getTime(*m_start) - *m_totalPauseTime;
    else
        return 0;
}

double biorbd::utils::Timer::stop()
{
    if (*m_isStarted){
        *m_isStarted = false;
        return getLap();
    }
    else
        return 0;
}

void biorbd::utils::Timer::addPauseTime(){
    if (*m_isPaused){
        *m_totalPauseTime += getTime(*m_pauseTime);
        *m_pauseTime = std::clock();
    }
}

double biorbd::utils::Timer::getTime(const std::clock_t& timer){
    return static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
}
