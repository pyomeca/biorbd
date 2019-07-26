#ifndef S2M_BENCHMARK_H
#define S2M_BENCHMARK_H
#include <ctime>
#include "biorbdConfig.h"
#include "s2mTimer.h"
#include "s2mString.h"

class BIORBD_API s2mBenchmark
{
    public:
        s2mBenchmark();
        ~s2mBenchmark(){}


        // Timers
        void startTimer(const s2mString&, bool force=false); // Start a timer related to name
        void pauseTimer(const s2mString&); // Pause a timer
        void resumeTimer(const s2mString&); // Pause a timer
        double getLap(const s2mString&);
        double stopTimer(const s2mString&);
        static void wasteTime(double timeInSec);

        // Counters
        void addToCounter(const s2mString&); // Start a counter to count
        int getCount(const s2mString&); // Get number of counts



    protected:
        std::map<s2mString, s2mTimer> m_timers;
        std::map<s2mString, int> m_counts;

    private:
};

#endif // S2M_BENCHMARK_H
