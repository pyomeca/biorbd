#ifndef BIORBD_UTILS_TIMER_H
#define BIORBD_UTILS_TIMER_H

#include <memory>
#include <ctime>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
///
/// \brief Class timer
///
class BIORBD_API Timer
{
public:
    ///
    /// \brief Construct timer
    /// \param startNow To start now (default: false)
    ///
    Timer(bool startNow = false);
    ///
    /// \brief Start a timer
    ///
    void start();
    ///
    /// \brief If timer is started
    ///
    bool isStarted();
    ///
    /// \brief Pause timer (use resume to restart)
    ///
    void pause();
    ///
    /// \brief Restart a timer
    ///
    void resume();
    ///
    /// \brief Get Lap
    /// \return A lap
    ///
    double getLap();
    ///
    /// \brief Stop timer
    ///
    double stop();

protected:
    ///
    /// \brief To add pause time
    ///
    void addPauseTime();
    /// 
    /// \brief To get the time
    /// \param timer Timer to get the time from
    /// \return The time
    ///
    double getTime(const std::clock_t&timer);

    bool m_isStarted; ///< If timer is started
    bool m_isPaused; ///< If time is paused
    std::clock_t m_start; ///< The start time
    std::clock_t m_pauseTime; ///<The pause time
    double m_totalPauseTime; ///< The total pause time

};

}}

#endif // BIORBD_UTILS_TIMER_H
