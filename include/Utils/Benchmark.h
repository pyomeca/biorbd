#ifndef BIORBD_UTILS_BENCHMARK_H
#define BIORBD_UTILS_BENCHMARK_H

#include <map>
#include "biorbdConfig.h"
#include "Utils/Timer.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {

class BIORBD_API Benchmark
{
    public:
        Benchmark();
        ~Benchmark();


        // Timers
        void startTimer(const biorbd::utils::String&, bool force=false); // Start a timer related to name
        void pauseTimer(const biorbd::utils::String&); // Pause a timer
        void resumeTimer(const biorbd::utils::String&); // Pause a timer
        double getLap(const biorbd::utils::String&);
        double stopTimer(const biorbd::utils::String&);
        static void wasteTime(double timeInSec);

        // Counters
        void addToCounter(const biorbd::utils::String&); // Start a counter to count
        int getCount(const biorbd::utils::String&); // Get number of counts

    protected:
        std::map<biorbd::utils::String, biorbd::utils::Timer> m_timers;
        std::map<biorbd::utils::String, int> m_counts;

};

}}

#endif // BIORBD_UTILS_BENCHMARK_H
