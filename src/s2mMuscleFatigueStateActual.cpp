#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueStateActual.h"

s2mMuscleFatigueStateActual::s2mMuscleFatigueStateActual(const double &mA, const double &mF, const double &mR) :
    s2mMuscleFatigueState(mA,mF,mR),
    m_previousActiveFibers(0),
    m_previousFatiguedFibers(0),
    m_previousRestingFibers(1),
    m_activeFibersDot(0),
    m_fatiguedFibersDot(0),
    m_restingFibersDot(0)
{
}

s2mMuscleFatigueStateActual::~s2mMuscleFatigueStateActual()
{
    //dtor
}


//double s2mMuscleFatigueStateActual::timeDerivativeActivation(const s2mMuscleFatigueStateActual& s, const s2mMuscleCaracteristics& c, const bool alreadyNormalized){
//    return timeDerivativeActivation(s.excitation(), s.activation(), c, alreadyNormalized);
//}


//double s2mMuscleFatigueStateActual::timeDerivativeActivation(double e, double a, const s2mMuscleCaracteristics &c, const bool alreadyNormalized){
//    setExcitation(e);
//    setActivation(a);
//    return timeDerivativeActivation(c, alreadyNormalized);
//}

std::vector<double> s2mMuscleFatigueStateActual::timeDerivativeState(const s2mMuscleCaracteristics &c){
    std::vector<double> res(3);
//    double num;
//    if (alreadyNormalized)
//        num = m_excitation-m_activation; // numérateur
//    else
//        num = excitationNorm(c.stateMax())-m_activation; // numérateur
    
//    double denom; // dénominateur
//    if (num>0)
//        denom = c.tauActivation()   * (0.5+1.5*m_activation);
//    else
//        denom = c.tauDeactivation() / (0.5+1.5*m_activation);

//    m_activationDot = num/denom;

    return res;
}

void s2mMuscleFatigueStateActual::setActiveFibers(const double &val) {
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

void s2mMuscleFatigueStateActual::setFatiguedFibers(const double &val) {
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

void s2mMuscleFatigueStateActual::setRestingFibers(const double &val) {
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

