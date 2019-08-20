#define BIORBD_API_EXPORTS
#include "Utils/Time.h"

biorbd::utils::Time::Time(
        double timeStep,
        unsigned int nbSteps)
{
    //ctor
    m_nbSteps = nbSteps;
    for (unsigned int i=0; i<nbSteps; i++)
        m_time.push_back(timeStep*i);
}

double biorbd::utils::Time::time(unsigned int t)
{
    return m_time.at(t);
}
