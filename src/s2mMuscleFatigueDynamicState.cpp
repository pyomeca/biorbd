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

s2mMuscleFatigueDynamicState::s2mMuscleFatigueDynamicState(const std::shared_ptr<s2mMuscleFatigueState> m) :
    s2mMuscleFatigueState(m)
{
    std::shared_ptr<s2mMuscleFatigueDynamicState> m_tp(std::dynamic_pointer_cast<s2mMuscleFatigueDynamicState>(m));
    if (!m_tp)
        s2mError::s2mAssert(false, "This is not a dynamically fatigable muscle");
    m_activeFibersDot = m_tp->m_activeFibersDot;
    m_fatiguedFibersDot = m_tp->m_fatiguedFibersDot;
    m_restingFibersDot = m_tp->m_restingFibersDot;
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

s2mVector s2mMuscleFatigueDynamicState::getTimeDerivativeState() const
{
   s2mVector res(3);
   res(0) = m_activeFibersDot;
   res(1) = m_fatiguedFibersDot;
   res(2) = m_restingFibersDot;
   return res;
}

void s2mMuscleFatigueDynamicState::setType()
{
    m_type = "DynamicAbstract";
}

