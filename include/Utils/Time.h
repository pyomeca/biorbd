#ifndef BIORBD_UTILS_TIME_H
#define BIORBD_UTILS_TIME_H

#include "biorbdConfig.h"

namespace biorbd { namespace utils {

class BIORBD_API Time
{
public:
    Time(
            const double &timeStep,
            const unsigned int &nbSteps);
    virtual ~Time();

    double time(const unsigned int &t); // Return time at index t
protected:
    double * m_time;
    unsigned int m_nbSteps;

};

}}

#endif // BIORBD_UTILS_TIME_H
