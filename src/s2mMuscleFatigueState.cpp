#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueState.h"

s2mMuscleFatigueState::s2mMuscleFatigueState(const double &mA, const double &mF, const double &mR) :
    m_activeFibers(mA),
    m_fatiguedFibers(mF),
    m_restingFibers(mR)

{
}

s2mMuscleFatigueState::~s2mMuscleFatigueState()
{
    //dtor
}

void s2mMuscleFatigueState::setState(const double &mA, const double &mF, const double &mR)
{
    if (mA<0){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be lower than 0, 0 is used then");
        m_activeFibers = 0;
    }
    else if (mA>1){
        s2mError::s2mWarning(0, "Active Fibers Quantity can't be higher than 1, 1 is used then");
        m_activeFibers = 1;
    }
    else {
        m_activeFibers = mA;
    }

    if (mR<0){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be lower than 0, 0 is used then");
        m_restingFibers = 0;
    }
    else if (mR>1){
        s2mError::s2mWarning(0, "Resting Fibers Quantity can't be higher than 1, 1 is used then");
        m_restingFibers = 1;
    }
    else {
        m_restingFibers = mR;
    }

    if (mF<0){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be lower than 0, 0 is used then");
        m_fatiguedFibers = 0;
    }
    else if (mF>1){
        s2mError::s2mWarning(0, "Fatigued Fibers Quantity can't be higher than 1, 1 is used then");
        m_fatiguedFibers = 1;
    }
    else {
        m_fatiguedFibers = mF;
    }

    if (m_activeFibers + m_restingFibers + m_fatiguedFibers > 1){
        s2mError::s2mAssert(0, "Sum of different state of muscle fatigue is higher than 1, it must be equal to 1");
    }
    else if (m_activeFibers + m_restingFibers + m_fatiguedFibers < 1){
        s2mError::s2mAssert(0, "Sum of different state of muscle fatigue is lower than 1, it must be equal to 1");
    }

}
//TODO one set fit all



double s2mMuscleFatigueState::activeFibers() const
{
    return m_activeFibers;
}

double s2mMuscleFatigueState::fatiguedFibers() const
{
    return m_fatiguedFibers;
}

double s2mMuscleFatigueState::restingFibers() const
{
    return m_restingFibers;
}
