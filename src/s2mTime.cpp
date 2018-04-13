#include "../include/s2mTime.h"

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
