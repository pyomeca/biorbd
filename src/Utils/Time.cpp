#define BIORBD_API_EXPORTS
#include "Utils/Time.h"

biorbd::utils::Time::Time() :
    m_time(std::vector<double>())
{

}

biorbd::utils::Time::Time(
        double timeStep,
        unsigned int nbSteps) :
    m_time(std::vector<double>())
{
    //ctor
    for (unsigned int i=0; i<nbSteps; i++)
        m_time.push_back(timeStep*i);
}

double biorbd::utils::Time::time(unsigned int t)
{
    return m_time.at(t);
}
