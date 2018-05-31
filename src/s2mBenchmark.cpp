#define BIORBD_API_EXPORTS
#include "../include/s2mBenchmark.h"

s2mBenchmark::s2mBenchmark(){

}

void s2mBenchmark::startTimer(const s2mString& s, bool force){
    if (force)
        m_timers[s].start();
    else
        if (!m_timers[s].isStarted())
            m_timers[s].start();
}

double s2mBenchmark::getLap(const s2mString& s){
    return m_timers[s].getLap();
}

double s2mBenchmark::stopTimer(const s2mString& s){
    return m_timers[s].stop();
}

void s2mBenchmark::pauseTimer(const s2mString& s){
    m_timers[s].pause();
}

void s2mBenchmark::resumeTimer(const s2mString& s){
    m_timers[s].resume();
}

void s2mBenchmark::addToCounter(const s2mString& s){
    ++(m_counts[s]);
}

int s2mBenchmark::getCount(const s2mString& s){
    return m_counts[s];
}

void s2mBenchmark::wasteTime(double seconds){
    // Wait for seconds ask doing dummy stuff

    std::clock_t start = std::clock();

    while ((((double)(std::clock() - start)) / CLOCKS_PER_SEC)<seconds)
    {
    }


}
