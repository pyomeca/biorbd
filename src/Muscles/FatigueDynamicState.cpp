#define BIORBD_API_EXPORTS
#include "Muscles/FatigueDynamicState.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

muscles::FatigueDynamicState::FatigueDynamicState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) :
    muscles::FatigueState(active,fatigued,resting),
    m_activeFibersDot(std::make_shared<utils::Scalar>(0)),
    m_fatiguedFibersDot(std::make_shared<utils::Scalar>(0)),
    m_restingFibersDot(std::make_shared<utils::Scalar>(0))
{
    setType();
}

muscles::FatigueDynamicState::FatigueDynamicState(
    const std::shared_ptr<muscles::FatigueState> other) :
    muscles::FatigueState(other)
{
    std::shared_ptr<muscles::FatigueDynamicState> muscle_tp(
        std::dynamic_pointer_cast<muscles::FatigueDynamicState>(other));
    if (!muscle_tp) {
        utils::Error::raise("This is not a dynamically fatigable muscle");
    }
    m_activeFibersDot = muscle_tp->m_activeFibersDot;
    m_fatiguedFibersDot = muscle_tp->m_fatiguedFibersDot;
    m_restingFibersDot = muscle_tp->m_restingFibersDot;
}

void muscles::FatigueDynamicState::DeepCopy(const
        muscles::FatigueDynamicState &other)
{
    muscles::FatigueState::DeepCopy(other);
    *m_activeFibersDot = *other.m_activeFibersDot;
    *m_fatiguedFibersDot = *other.m_fatiguedFibersDot;
    *m_restingFibersDot = *other.m_restingFibersDot;
}

const utils::Scalar&
muscles::FatigueDynamicState::activeFibersDot() const
{
    return *m_activeFibersDot;
}

const utils::Scalar&
muscles::FatigueDynamicState::fatiguedFibersDot() const
{
    return *m_fatiguedFibersDot;
}

const utils::Scalar&
muscles::FatigueDynamicState::restingFibersDot() const
{
    return *m_restingFibersDot;
}

