#define BIORBD_API_EXPORTS
#include "Utils/Benchmark.h"

#include "Utils/Timer.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

utils::Benchmark::Benchmark() :
    m_timers(std::map<utils::String, utils::Timer>()),
    m_counts(std::map<utils::String, int>())
{

}


void utils::Benchmark::startTimer(
    const utils::String& name,
    bool force)
{
    if (force) {
        m_timers[name].start();
    } else if (!m_timers[name].isStarted()) {
        m_timers[name].start();
    }
}

double utils::Benchmark::getLap(
    const utils::String& name)
{
    return m_timers[name].getLap();
}

double utils::Benchmark::stopTimer(
    const utils::String& name)
{
    return m_timers[name].stop();
}

void utils::Benchmark::pauseTimer(
    const utils::String& name)
{
    m_timers[name].pause();
}

void utils::Benchmark::resumeTimer(
    const utils::String& name)
{
    m_timers[name].resume();
}

void utils::Benchmark::addTimer(
    const utils::String& name)
{
    ++(m_counts[name]);
}

int utils::Benchmark::getTimerIdx(
    const utils::String& name)
{
    return m_counts[name];
}

void utils::Benchmark::wasteTime(
    double seconds)
{
    // Wait for seconds ask doing dummy stuff

    std::clock_t start(std::clock());

    while (((static_cast<double>(std::clock() - start)) / CLOCKS_PER_SEC)<seconds) {
    }

}
