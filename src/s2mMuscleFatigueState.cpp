#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueState.h"

s2mMuscleFatigueState::s2mMuscleFatigueState(const double &mA, const double &mF, const double &mR) :
    m_activeFibers(mA),
    m_fatiguedFibers(mF),
    m_restingFibers(mR)

{
}

s2mMuscleFatigueState::~s2mMuscleFatigueState()
{
    //dtor
}

void s2mMuscleFatigueState::setActiveFibers(const double &val) {
    if (m_activeFibers<=0)
        m_activeFibers = 0;
    else if (m_activeFibers>=1)
        m_activeFibers = 1;
    else
        m_activeFibers = val;
}

void s2mMuscleFatigueState::setFatiguedFibers(const double &val) {
    if (m_fatiguedFibers<=0)
        m_fatiguedFibers = 0;
    else if (m_fatiguedFibers>=1)
        m_fatiguedFibers = 1;
    else
        m_fatiguedFibers = val;
}

void s2mMuscleFatigueState::setRestingFibers(const double &val) {
    if (m_restingFibers<=0)
        m_restingFibers = 0;
    else if (m_restingFibers>=1)
        m_restingFibers = 1;
    else
        m_restingFibers = val;
}

double s2mMuscleFatigueState::activeFibers() const
{
    return m_activeFibers;
}

double s2mMuscleFatigueState::fatiguedFibers() const
{
    return m_fatiguedFibers;
}

double s2mMuscleFatigueState::restingFibers() const
{
    return m_restingFibers;
}
