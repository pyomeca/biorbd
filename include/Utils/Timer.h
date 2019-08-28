#ifndef BIORBD_UTILS_TIMER_H
#define BIORBD_UTILS_TIMER_H

#include <memory>
#include <ctime>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {

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

    std::shared_ptr<bool> m_isStarted;
    std::shared_ptr<bool> m_isPaused;
    std::shared_ptr<std::clock_t> m_start;
    std::shared_ptr<std::clock_t> m_pauseTime;
    std::shared_ptr<double> m_totalPauseTime;

};

}}

#endif // BIORBD_UTILS_TIMER_H
