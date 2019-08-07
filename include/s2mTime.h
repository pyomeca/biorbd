#define S2M_TIME_H
#ifndef S2M_TIME_H

#include "biorbdConfig.h"

class BIORBD_API s2mTime
{
    public:
        s2mTime(const double &timeStep, const unsigned int &nbSteps);
        virtual ~s2mTime();

        double time(const unsigned int &t); // Return time at index t
    protected:
        double * m_time;
        unsigned int m_nbSteps;

};

#endif // S2M_TIME_H
