#ifndef S2M_TIMER_H
#define S2M_TIMER_H
#include <ctime>
#include <iostream>
#include "biorbdConfig.h"
    
class BIORBD_API s2mTimer
{
    public:
    s2mTimer(bool startNow = false);
        ~s2mTimer(){}

        void start(); // Start a timer
        bool isStarted();
        void pause(); // Pause timer, use resume to restart
        void resume(); // Restart a timer
        double getLap();
        double stop();

    protected:
        void addPauseTime();
        double getTime(const std::clock_t&);

        bool m_isStarted;
        bool m_isPaused;
        std::clock_t m_start;
        std::clock_t m_pauseTime;
        double m_totalPauseTime;

    private:
};

#endif // S2M_TIMER_H
