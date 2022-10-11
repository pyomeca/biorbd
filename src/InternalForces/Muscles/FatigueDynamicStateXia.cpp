#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueDynamicStateXia.h"

#include <cmath>
#include "Utils/Error.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/FatigueParameters.h"
#include "InternalForces/Muscles/StateDynamics.h"

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) :
    muscles::FatigueDynamicState(active,fatigued,resting)
{
    setType();
}

muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const std::shared_ptr<muscles::FatigueState> other):
    muscles::FatigueDynamicState(other)
{

}

muscles::FatigueDynamicStateXia
muscles::FatigueDynamicStateXia::DeepCopy() const
{
    muscles::FatigueDynamicStateXia copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::FatigueDynamicStateXia::DeepCopy(const
        muscles::FatigueDynamicStateXia &other)
{
    muscles::FatigueDynamicState::DeepCopy(other);
}

void muscles::FatigueDynamicStateXia::timeDerivativeState(
    const muscles::StateDynamics &emg,
    const muscles::Characteristics &characteristics)
{
#ifdef BIORBD_USE_CASADI_MATH
    utils::Error::raise("timeDerivativeState for FatigueDynamicStateXia"
                                " is not implemented yet");
#else
    // Getting the command
    utils::Scalar targetCommand(emg.activation());
    utils::Scalar command(0);
    if (*m_activeFibers < targetCommand) {
        if (*m_restingFibers > targetCommand - *m_activeFibers) {
            command = characteristics.fatigueParameters().developFactor()*
                      (targetCommand - *m_activeFibers);
        } else {
            command = characteristics.fatigueParameters().developFactor()* *m_restingFibers;
        }
    } else {
        command = characteristics.fatigueParameters().recoveryFactor()*
                  (targetCommand - *m_activeFibers);
    }

    // Applying the command to the fibers
    *m_activeFibersDot = command - characteristics.fatigueParameters().fatigueRate()
                         * *m_activeFibers;
    *m_restingFibersDot = -command +
                          characteristics.fatigueParameters().recoveryRate()* *m_fatiguedFibers;
    *m_fatiguedFibersDot = characteristics.fatigueParameters().fatigueRate()*
                           *m_activeFibers -
                           characteristics.fatigueParameters().recoveryRate()* *m_fatiguedFibers;

    utils::Error::check(
        fabs(*m_activeFibersDot + *m_restingFibersDot + *m_fatiguedFibersDot) <= 1e-7,
        "Sum of time derivates of fatigue states must be equal to 0");
#endif
}

void muscles::FatigueDynamicStateXia::setType()
{
    *m_type =muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA;
}
