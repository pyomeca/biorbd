#ifndef BIORBD_UTILS_BENCHMARK_H
#define BIORBD_UTILS_BENCHMARK_H

#include <memory>
#include <map>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Timer;
///
/// \brief Class benchmark
///
class BIORBD_API Benchmark
{
public:
    ///
    /// \brief Construct benchmark
    ///
    Benchmark();

    // Timers
    /// 
    /// \brief Start a timer related to name
    /// \param s The name to associate with timer
    /// \param force If it is a force (default: false)
    ///
    void startTimer(
            const biorbd::utils::String&s,
            bool force=false);

    /// 
    /// \brief Pause a timer
    /// \param s Name of the timer to pause
    ///
    void pauseTimer(const biorbd::utils::String&s);

    /// 
    /// \brief Resume a timer
    /// \param s Name of the timer to pause
    ///
    void resumeTimer(const biorbd::utils::String&s); 

    ///
    /// \brief Get lap
    /// \param s Name of the timer
    /// \return The lap
    ///
    double getLap(const biorbd::utils::String&s);

    ///
    /// \brief Stop the timer and get lap
    /// \param s Name of the timer
    /// \return The lap
    ///
    double stopTimer(const biorbd::utils::String&s);

    ///
    /// \brief To waste time (Wait for seconds ask doing dummy stuff)
    /// \param timeInSec Time in seconds
    ///
    static void wasteTime(double timeInSec);

    // Counters
    ///
    /// \brief Start a counter to count or add to it
    /// \param s Counter to add
    ///
    void addToCounter(const biorbd::utils::String&);

    ///
    /// \brief Get the number of counts
    /// \param s The count to get
    /// \return The number of counts
    ///
    int getCount(const biorbd::utils::String&); 

protected:
    std::map<biorbd::utils::String, biorbd::utils::Timer> m_timers;///< Timers
    std::map<biorbd::utils::String, int> m_counts;///< Counts

};

}}

#endif // BIORBD_UTILS_BENCHMARK_H
