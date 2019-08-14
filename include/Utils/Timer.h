#ifndef BIORBD_UTILS_TIMER_H
#define BIORBD_UTILS_TIMER_H

#include <ctime>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {

class BIORBD_API Timer
{
public:
    Timer(bool startNow = false);
    virtual ~Timer();

    void start(); // Start a timer
    bool isStarted();
    void pause(); // Pause timer, use resume to restart
    void resume(); // Restart a timer
    double getLap();
    double stop();

protected:
    void addPauseTime();
    double getTime(const std::clock_t&);

    bool m_isStarted;
    bool m_isPaused;
    std::clock_t m_start;
    std::clock_t m_pauseTime;
    double m_totalPauseTime;

};

}}

#endif // BIORBD_UTILS_TIMER_H
