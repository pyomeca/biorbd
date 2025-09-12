#define BIORBD_API_EXPORTS
#include "Utils/Timer.h"

using namespace BIORBD_NAMESPACE;

utils::Timer::Timer(bool startNow)
    : m_isStarted(false),
      m_isPaused(false),
      m_start(),
      m_pauseTime(),
      m_totalPauseTime(0.0) {
  if (startNow) {
    start();
  }
}

void utils::Timer::start() {
  m_start = std::clock();
  m_totalPauseTime = 0;
  m_isPaused = false;
  m_isStarted = true;
}  // Start a timer

bool utils::Timer::isStarted() { return m_isStarted; }

void utils::Timer::pause() {
  if (!m_isPaused) {
    m_isPaused = true;
    m_pauseTime = std::clock();
  }
}

void utils::Timer::resume() {
  if (!m_isStarted) {
    start();
  }

  else if (m_isPaused) {
    addPauseTime();
    m_isPaused = false;
  }
}

double utils::Timer::getLap() {
  addPauseTime();

  if (m_isStarted) {
    return getTime(m_start) - m_totalPauseTime;
  } else {
    return 0;
  }
}

double utils::Timer::stop() {
  if (m_isStarted) {
    m_isStarted = false;
    return getLap();
  } else {
    return 0;
  }
}

void utils::Timer::addPauseTime() {
  if (m_isPaused) {
    m_totalPauseTime += getTime(m_pauseTime);
    m_pauseTime = std::clock();
  }
}

double utils::Timer::getTime(const std::clock_t& timer) {
  return static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
}
