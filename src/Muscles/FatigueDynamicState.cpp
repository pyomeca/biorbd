#define BIORBD_API_EXPORTS
#include "Muscles/FatigueDynamicState.h"

#include "Utils/Error.h"

biorbd::muscles::FatigueDynamicState::FatigueDynamicState(
        double active,
        double fatigued,
        double resting) :
    biorbd::muscles::FatigueState(active,fatigued,resting),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
    setType();
}

biorbd::muscles::FatigueDynamicState::FatigueDynamicState(const std::shared_ptr<biorbd::muscles::FatigueState> m) :
    biorbd::muscles::FatigueState(m)
{
    std::shared_ptr<biorbd::muscles::FatigueDynamicState> m_tp(std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>(m));
    if (!m_tp)
        biorbd::utils::Error::error(false, "This is not a dynamically fatigable muscle");
    m_activeFibersDot = m_tp->m_activeFibersDot;
    m_fatiguedFibersDot = m_tp->m_fatiguedFibersDot;
    m_restingFibersDot = m_tp->m_restingFibersDot;
}

double biorbd::muscles::FatigueDynamicState::activeFibersDot() const
{
    return m_activeFibersDot;
}

double biorbd::muscles::FatigueDynamicState::fatiguedFibersDot() const
{
    return m_fatiguedFibersDot;
}

double biorbd::muscles::FatigueDynamicState::restingFibersDot() const
{
    return m_restingFibersDot;
}

void biorbd::muscles::FatigueDynamicState::setType()
{
    m_type = "DynamicAbstract";
}
