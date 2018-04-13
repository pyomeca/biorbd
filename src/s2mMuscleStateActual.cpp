#include "../include/s2mMuscleStateActual.h"

s2mMuscleStateActual::s2mMuscleStateActual(const double &e, const double &a) :
    s2mMuscleState(e,a),
    m_excitationNorm(0),
    m_previousExcitation(0),
    m_previousActivation(0),
    m_activationDot(0)
{
}

s2mMuscleStateActual::~s2mMuscleStateActual()
{
    //dtor
}


double s2mMuscleStateActual::timeDerivativeActivation(const s2mMuscleStateActual& s, const s2mMuscleCaracteristics& c, const bool alreadyNormalized){
    return timeDerivativeActivation(s.excitation(), s.activation(), c, alreadyNormalized);
}


double s2mMuscleStateActual::timeDerivativeActivation(double e, double a, const s2mMuscleCaracteristics &c, const bool alreadyNormalized){
    setExcitation(e);
    setActivation(a);
    return timeDerivativeActivation(c, alreadyNormalized);
}

double s2mMuscleStateActual::timeDerivativeActivation(const s2mMuscleCaracteristics &c, const bool alreadyNormalized){
    // Implémentation de la fonction da/dt = (u-a)/tau(u,a)
    // ou tau(u,a) = t_act(0.5+1.5*a) is u>a et tau(u,a)=t_deact(0.5+1.5*a) sinon
    if (m_activation<c.minActivation())
        m_activation = c.minActivation();

    if (m_excitation<c.minActivation())
        m_excitation = c.minActivation();



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    double num;
    if (alreadyNormalized)
        num = m_excitation-m_activation; // numérateur
    else
        num = excitationNorm(c.stateMax())-m_activation; // numérateur
    
    double denom; // dénominateur
    if (num>0)
        denom = c.tauActivation()   * (0.5+1.5*m_activation);
    else
        denom = c.tauDeactivation() / (0.5+1.5*m_activation);

    m_activationDot = num/denom;

	return m_activationDot;
}

void s2mMuscleStateActual::setExcitation(const double &val) {
    m_previousExcitation = m_excitation;
    if (val<0){
        s2mError::s2mWarning(0, "Excitation can't be lower than 0, 0 is used then");
        m_excitation = 0;
    }
    else
        m_excitation = val;
}
void s2mMuscleStateActual::setActivation(const double &val) {
    m_previousActivation = m_activation;

    if (val<0){
        s2mError::s2mWarning(0, "Activation can't be lower than 0, 0 is used then");
        m_activation = 0;
    }
    else
        m_activation = val;
}

double s2mMuscleStateActual::excitationNorm(const s2mMuscleState &max) {
    s2mError::s2mWarning(m_excitation<max.excitation(), "Excitation is higher than maximal excitation.");
    m_excitationNorm = m_excitation / max.excitation();
    
    return m_excitationNorm;
}
