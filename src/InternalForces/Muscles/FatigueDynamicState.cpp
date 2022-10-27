#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueDynamicState.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::FatigueDynamicState::FatigueDynamicState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) :
    internal_forces::muscles::FatigueState(active,fatigued,resting),
    m_activeFibersDot(std::make_shared<utils::Scalar>(0)),
    m_fatiguedFibersDot(std::make_shared<utils::Scalar>(0)),
    m_restingFibersDot(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::muscles::FatigueDynamicState::FatigueDynamicState(
    const std::shared_ptr<internal_forces::muscles::FatigueState> other) :
    internal_forces::muscles::FatigueState(other)
{
    std::shared_ptr<internal_forces::muscles::FatigueDynamicState> muscle_tp(
        std::dynamic_pointer_cast<internal_forces::muscles::FatigueDynamicState>(other));
    if (!muscle_tp) {
        utils::Error::raise("This is not a dynamically fatigable muscle");
    }
    m_activeFibersDot = muscle_tp->m_activeFibersDot;
    m_fatiguedFibersDot = muscle_tp->m_fatiguedFibersDot;
    m_restingFibersDot = muscle_tp->m_restingFibersDot;
}

void internal_forces::muscles::FatigueDynamicState::DeepCopy(const
        internal_forces::muscles::FatigueDynamicState &other)
{
    internal_forces::muscles::FatigueState::DeepCopy(other);
    *m_activeFibersDot = *other.m_activeFibersDot;
    *m_fatiguedFibersDot = *other.m_fatiguedFibersDot;
    *m_restingFibersDot = *other.m_restingFibersDot;
}

const utils::Scalar&
internal_forces::muscles::FatigueDynamicState::activeFibersDot() const
{
    return *m_activeFibersDot;
}

const utils::Scalar&
internal_forces::muscles::FatigueDynamicState::fatiguedFibersDot() const
{
    return *m_fatiguedFibersDot;
}

const utils::Scalar&
internal_forces::muscles::FatigueDynamicState::restingFibersDot() const
{
    return *m_restingFibersDot;
}

