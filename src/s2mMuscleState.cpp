#include "../include/s2mMuscleState.h"

s2mMuscleState::s2mMuscleState(const double &e, const double &a) :
    m_excitation(e),
    m_activation(a)
{
}

s2mMuscleState::~s2mMuscleState()
{
    //dtor
}

void s2mMuscleState::setExcitation(const double &val) {
    if (m_excitation<=0)
        m_excitation = 0;
    else
        m_excitation = val;
}

void s2mMuscleState::setActivation(const double &val){
    if (m_activation<=0)
        m_activation = 0;
    else if (m_activation>=1)
        m_activation = 1;
    else
        m_activation = val;
}

double s2mMuscleState::excitation() const
{
    return m_excitation;
}

double s2mMuscleState::activation() const
{
    return m_activation;
}
