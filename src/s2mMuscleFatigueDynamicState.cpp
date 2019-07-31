#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueDynamicState.h"

s2mMuscleFatigueDynamicState::s2mMuscleFatigueDynamicState(
        double active,
        double fatigued,
        double resting) :
    s2mMuscleFatigueState(active,fatigued,resting),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
    setType();
}

s2mMuscleFatigueDynamicState::s2mMuscleFatigueDynamicState(
        const std::shared_ptr<s2mMuscle> m):
    s2mMuscleFatigueState(),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
    setType();
}

double s2mMuscleFatigueDynamicState::activeFibersDot() const
{
    return m_activeFibersDot;
}

double s2mMuscleFatigueDynamicState::fatiguedFibersDot() const
{
    return m_fatiguedFibersDot;
}

double s2mMuscleFatigueDynamicState::restingFibersDot() const
{
    return m_restingFibersDot;
}

void s2mMuscleFatigueDynamicState::setType()
{
    m_type = "DynamicAbstract";
}

