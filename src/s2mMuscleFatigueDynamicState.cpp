#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueDynamicState.h"

s2mMuscleFatigueDynamicState::s2mMuscleFatigueDynamicState(
        double active,
        double fatigued,
        double resting) :
    s2mMuscleFatigueState(active,fatigued,resting),
    m_previousActiveFibers(0),
    m_previousFatiguedFibers(0),
    m_previousRestingFibers(1),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
    setType();
}

double s2mMuscleFatigueDynamicState::previousActiveFibers() const {
    return m_previousActiveFibers;
}

double s2mMuscleFatigueDynamicState::previousFatiguedFibers() const {
    return m_previousFatiguedFibers;
}

double s2mMuscleFatigueDynamicState::previousRestingFibers() const {
    return m_previousRestingFibers;
}

void s2mMuscleFatigueDynamicState::setType()
{
    m_type = "DynamicAbstract";
}

