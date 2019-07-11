#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueStateActualXia.h"

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

std::vector<double> s2mMuscleFatigueStateActualXia::timeDerivativeState(const s2mMuscleStateActual &EMG, const s2mMuscleCaracteristics &c){
    std::vector<double> res(3);
    double command(0);
    double targetCommand(EMG.activation());
    if (m_activeFibers < targetCommand){
        if (m_restingFibers > targetCommand - m_activeFibers){
            command = c.fatigueParameters().developFactor()*(targetCommand - m_activeFibers);
        }
        else {
            command = c.fatigueParameters().developFactor()*m_restingFibers;
        }
    }
    else {
        command = c.fatigueParameters().recoverFactor()*(targetCommand - m_activeFibers);
    }
    m_activeFibersDot = command - c.fatigueParameters().fatigueRate()*m_activeFibers;
    m_restingFibersDot = -command + c.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    m_fatiguedFibersDot = c.fatigueParameters().fatigueRate()*m_activeFibers - c.fatigueParameters().recoveryRate()*m_fatiguedFibers;
    res[0] = m_activeFibers;
    res[1] = m_fatiguedFibers;
    res[2] = m_restingFibers;
    return res;
}

void s2mMuscleFatigueStateActualXia::setActiveFibers(const double &val) {
    m_previousActiveFibers = m_activeFibers;
    if (val<0){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then");
        m_activeFibers = 0;
    }
    else if (val>1){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then");
        m_activeFibers = 1;
    }
    else
        m_activeFibers = val;
}

void s2mMuscleFatigueStateActualXia::setFatiguedFibers(const double &val) {
    m_previousFatiguedFibers = m_fatiguedFibers;
    if (val<0){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be lower than 0, 0 is used then");
        m_fatiguedFibers = 0;
    }
    else if (val>1){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be higher than 1, 1 is used then");
        m_fatiguedFibers = 1;
    }
    else
        m_fatiguedFibers = val;
}

void s2mMuscleFatigueStateActualXia::setRestingFibers(const double &val) {
    m_previousRestingFibers = m_restingFibers;
    if (val<0){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then");
        m_restingFibers = 0;
    }
    else if (val>1){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be higher than 1, 1 is used then");
        m_restingFibers = 1;
    }
    else
        m_restingFibers = val;
}

