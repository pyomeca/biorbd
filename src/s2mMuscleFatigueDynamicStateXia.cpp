#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueDynamicStateXia.h"

s2mMuscleFatigueStateActualXia::s2mMuscleFatigueStateActualXia(
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

s2mVector s2mMuscleFatigueStateActualXia::timeDerivativeState(const s2mMuscleStateActual &EMG, const s2mMuscleCaracteristics &caract){
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
        command = caract.fatigueParameters().recoverFactor()*(targetCommand - m_activeFibers);
    }

    // Applying the command to the fibers
    m_activeFibersDot = command - caract.fatigueParameters().fatigueRate()*m_activeFibers;
    m_restingFibersDot = -command + caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    m_fatiguedFibersDot = caract.fatigueParameters().fatigueRate()*m_activeFibers - caract.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    s2mError::s2mAssert(m_activeFibersDot + m_restingFibersDot + m_fatiguedFibersDot != 0.0, "Sum of time derivates of fatigue states must be equal to 0");

    // output results
    s2mVector res(3);
    res[0] = m_activeFibers;
    res[1] = m_fatiguedFibers;
    res[2] = m_restingFibers;
    return res;
}

double s2mMuscleFatigueStateActualXia::previousActiveFibers() const {
    return m_previousActiveFibers;
}

double s2mMuscleFatigueStateActualXia::previousFatiguedFibers() const {
    return m_previousFatiguedFibers;
}

double s2mMuscleFatigueStateActualXia::previousRestingFibers() const {
    return m_previousRestingFibers;
}

