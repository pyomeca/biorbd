#ifndef S2MTIME_H
#define S2MTIME_H
#include "biorbdConfig.h"


class BIORBD_API s2mTime
{
    public:
        s2mTime(const double &timeStep, const unsigned int &nbSteps);
        ~s2mTime();

        double time(const unsigned int &t); // Return time at index t
    protected:
        double * m_time;
        unsigned int m_nbSteps;

    private:
};

#endif // S2MTIME_H
