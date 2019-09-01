#ifndef BIORBD_UTILS_BENCHMARK_H
#define BIORBD_UTILS_BENCHMARK_H

#include <memory>
#include <map>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Timer;

class BIORBD_API Benchmark
{
public:
    Benchmark();

    // Timers
    void startTimer(
            const biorbd::utils::String&,
            bool force=false); // Start a timer related to name
    void pauseTimer(const biorbd::utils::String&); // Pause a timer
    void resumeTimer(const biorbd::utils::String&); // Pause a timer
    double getLap(const biorbd::utils::String&);
    double stopTimer(const biorbd::utils::String&);
    static void wasteTime(double timeInSec);
    biorbd::utils::Benchmark DeepCopy() const;
    void DeepCopy(const biorbd::utils::Benchmark& other);

    // Counters
    void addToCounter(const biorbd::utils::String&); // Start a counter to count
    int getCount(const biorbd::utils::String&); // Get number of counts

protected:
    std::shared_ptr<std::map<biorbd::utils::String, biorbd::utils::Timer>> m_timers;
    std::shared_ptr<std::map<biorbd::utils::String, int>> m_counts;

};

}}

#endif // BIORBD_UTILS_BENCHMARK_H
