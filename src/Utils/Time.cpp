#define BIORBD_API_EXPORTS
#include "Utils/Time.h"

s2mTime::s2mTime(const double &timeStep, const unsigned int &nbSteps)
{
    //ctor
    m_nbSteps = nbSteps;
    m_time = new double[nbSteps];
    for (unsigned int i=0; i<nbSteps; i++)
        m_time[i] =  timeStep*i;
}

s2mTime::~s2mTime()
{
    //dtor
    delete[] m_time;
}

double s2mTime::time(const unsigned int &t)
{
    if (t>=m_nbSteps)
        return 0;
    else
        return m_time[t];
}
