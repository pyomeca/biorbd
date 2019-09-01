#define BIORBD_API_EXPORTS
#include "Utils/Time.h"

biorbd::utils::Time::Time() :
    m_time(std::make_shared<std::vector<double>>())
{

}

biorbd::utils::Time::Time(
        double timeStep,
        unsigned int nbSteps) :
    m_time(std::make_shared<std::vector<double>>())
{
    //ctor
    for (unsigned int i=0; i<nbSteps; i++)
        m_time->push_back(timeStep*i);
}

biorbd::utils::Time biorbd::utils::Time::DeepCopy() const
{
    biorbd::utils::Time copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Time::DeepCopy(const biorbd::utils::Time &other)
{
    m_time->resize(other.m_time->size());
    for (unsigned int i=0; i<other.m_time->size(); ++i)
        (*m_time)[i] = (*other.m_time)[i];
}

double biorbd::utils::Time::time(unsigned int t)
{
    return m_time->at(t);
}
