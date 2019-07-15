#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueDynamicStateXia.h"

s2mMuscleFatigueStateActualXia::s2mMuscleFatigueStateActualXia(const double &mA, const double &mF, const double &mR) :
    s2mMuscleFatigueState(mA,mF,mR),
    m_previousActiveFibers(0),
    m_previousFatiguedFibers(0),
    m_previousRestingFibers(1),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
}

s2mMuscleFatigueStateActualXia::~s2mMuscleFatigueStateActualXia()
{
    //dtor
}

s2mVector s2mMuscleFatigueStateActualXia::timeDerivativeState(const s2mMuscleStateActual &EMG, const s2mMuscleCaracteristics &c){
    s2mVector res(3);
    double command(0);
    double targetCommand(EMG.activation());
    if (m_activeFibers < targetCommand){
        if (m_restingFibers > targetCommand - m_activeFibers){
            command = c.fatigueParameters().developFactor()*(targetCommand - m_activeFibers);
        } else {
            command = c.fatigueParameters().developFactor()*m_restingFibers;
        }
    } else {
        command = c.fatigueParameters().recoverFactor()*(targetCommand - m_activeFibers);
    }
    m_activeFibersDot = command - c.fatigueParameters().fatigueRate()*m_activeFibers;
    m_restingFibersDot = -command + c.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    m_fatiguedFibersDot = c.fatigueParameters().fatigueRate()*m_activeFibers - c.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    s2mError::s2mAssert(m_activeFibersDot + m_restingFibersDot + m_fatiguedFibersDot == 0.0, "Sum of time derivates of fatigue states must be equal to 0");
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

double s2mMuscleFatigueStateActualXia::previousRestingFibers() const { return m_previousRestingFibers; }

