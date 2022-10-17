#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueDynamicStateXia.h"

#include <cmath>
#include "Utils/Error.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/FatigueParameters.h"
#include "InternalForces/Muscles/StateDynamics.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) :
    internalforce::muscles::FatigueDynamicState(active,fatigued,resting)
{
    setType();
}

internalforce::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const std::shared_ptr<internalforce::muscles::FatigueState> other):
    internalforce::muscles::FatigueDynamicState(other)
{

}

internalforce::muscles::FatigueDynamicStateXia
internalforce::muscles::FatigueDynamicStateXia::DeepCopy() const
{
    internalforce::muscles::FatigueDynamicStateXia copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::FatigueDynamicStateXia::DeepCopy(const
        internalforce::muscles::FatigueDynamicStateXia &other)
{
    internalforce::muscles::FatigueDynamicState::DeepCopy(other);
}

void internalforce::muscles::FatigueDynamicStateXia::timeDerivativeState(
    const internalforce::muscles::StateDynamics &emg,
    const internalforce::muscles::Characteristics &characteristics)
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

void internalforce::muscles::FatigueDynamicStateXia::setType()
{
    *m_type =internalforce::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA;
}
