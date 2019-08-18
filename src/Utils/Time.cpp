#define BIORBD_API_EXPORTS
#include "Utils/Time.h"

biorbd::utils::Time::Time(
        double timeStep,
        unsigned int nbSteps)
{
    //ctor
    m_nbSteps = nbSteps;
    m_time = new double[nbSteps];
    for (unsigned int i=0; i<nbSteps; i++)
        m_time[i] =  timeStep*i;
}

biorbd::utils::Time::~Time()
{
    //dtor
    delete[] m_time;
}

double biorbd::utils::Time::time(unsigned int t)
{
    if (t>=m_nbSteps)
        return 0;
    else
        return m_time[t];
}
