#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueDynamicStateXia.h"

s2mMuscleFatigueDynamicStateXia::s2mMuscleFatigueDynamicStateXia(
        double active,
        double fatigued,
        double resting) :
    s2mMuscleFatigueDynamicState(active,fatigued,resting)
{
    setType();
}

void s2mMuscleFatigueDynamicStateXia::timeDerivativeState(const s2mMuscleStateActual &EMG, const s2mMuscleCaracteristics &caract){
    // Getting the command
    double targetCommand(EMG.activation());
    double command(0);
    if (m_activeFibers < targetCommand){
        if (m_restingFibers > targetCommand - m_activeFibers){
            command = caract.fatigueParameters().developFactor()*(targetCommand - m_activeFibers);
        } else {
            command = caract.fatigueParameters().developFactor()*m_restingFibers;
        }
    } else {
        command = caract.fatigueParameters().recoveryFactor()*(targetCommand - m_activeFibers);
    }

    // Applying the command to the fibers
    m_activeFibersDot = command - caract.fatigueParameters().fatigueRate()*m_activeFibers;
    m_restingFibersDot = -command + caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    m_fatiguedFibersDot = caract.fatigueParameters().fatigueRate()*m_activeFibers - caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    s2mError::s2mAssert(abs(m_activeFibersDot + m_restingFibersDot + m_fatiguedFibersDot) <= 1e-7, "Sum of time derivates of fatigue states must be equal to 0");
}

void s2mMuscleFatigueDynamicStateXia::setType()
{
    m_type = "Xia";
}
