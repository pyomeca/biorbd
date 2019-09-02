#define BIORBD_API_EXPORTS
#include "Utils/Benchmark.h"

#include "Utils/Timer.h"
#include "Utils/String.h"

biorbd::utils::Benchmark::Benchmark() :
    m_timers(std::map<biorbd::utils::String, biorbd::utils::Timer>()),
    m_counts(std::map<biorbd::utils::String, int>())
{

}


void biorbd::utils::Benchmark::startTimer(const biorbd::utils::String& s, bool force){
    if (force)
        m_timers[s].start();
    else
        if (!m_timers[s].isStarted())
            m_timers[s].start();
}

double biorbd::utils::Benchmark::getLap(const biorbd::utils::String& s){
    return m_timers[s].getLap();
}

double biorbd::utils::Benchmark::stopTimer(const biorbd::utils::String& s){
    return m_timers[s].stop();
}

void biorbd::utils::Benchmark::pauseTimer(const biorbd::utils::String& s){
    m_timers[s].pause();
}

void biorbd::utils::Benchmark::resumeTimer(const biorbd::utils::String& s){
    m_timers[s].resume();
}

void biorbd::utils::Benchmark::addToCounter(const biorbd::utils::String& s){
    ++(m_counts[s]);
}

int biorbd::utils::Benchmark::getCount(const biorbd::utils::String& s){
    return m_counts[s];
}

void biorbd::utils::Benchmark::wasteTime(double seconds){
    // Wait for seconds ask doing dummy stuff

    std::clock_t start(std::clock());

    while (((static_cast<double>(std::clock() - start)) / CLOCKS_PER_SEC)<seconds)
    {
    }

}
