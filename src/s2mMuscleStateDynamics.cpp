#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleStateDynamics.h"

s2mMuscleStateDynamics::s2mMuscleStateDynamics(const double &e, const double &a) :
    s2mMuscleState(e,a),
    m_excitationNorm(0),
    m_previousExcitation(0),
    m_previousActivation(0),
    m_activationDot(0)
{
}

s2mMuscleStateDynamics::~s2mMuscleStateDynamics()
{
    //dtor
}


double s2mMuscleStateDynamics::timeDerivativeActivation(const s2mMuscleStateDynamics& state, const s2mMuscleCaracteristics& caract, const bool alreadyNormalized){
    return timeDerivativeActivation(state.excitation(), state.activation(), caract, alreadyNormalized);
}


double s2mMuscleStateDynamics::timeDerivativeActivation(double excitation, double activation, const s2mMuscleCaracteristics &caract, const bool alreadyNormalized){
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(caract, alreadyNormalized);
}

double s2mMuscleStateDynamics::timeDerivativeActivation(const s2mMuscleCaracteristics &caract, const bool alreadyNormalized){
    // Implémentation de la fonction da/dt = (u-a)/tau(u,a)
    // ou tau(u,a) = t_act(0.5+1.5*a) is u>a et tau(u,a)=t_deact(0.5+1.5*a) sinon
    if (m_activation<caract.minActivation())
        m_activation = caract.minActivation();

    if (m_excitation<caract.minActivation())
        m_excitation = caract.minActivation();



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    double num;
    if (alreadyNormalized)
        num = m_excitation-m_activation; // numérateur
    else
        num = excitationNorm(caract.stateMax())-m_activation; // numérateur
    
    double denom; // dénominateur
    if (num>0)
        denom = caract.tauActivation()   * (0.5+1.5*m_activation);
    else
        denom = caract.tauDeactivation() / (0.5+1.5*m_activation);

    m_activationDot = num/denom;

	return m_activationDot;
}

double s2mMuscleStateDynamics::timeDerivativeActivation() {return m_activationDot;}

void s2mMuscleStateDynamics::setExcitation(const double &val) {
    m_previousExcitation = m_excitation;
    if (val<0){
        s2mError::s2mWarning(0, "Excitation can't be lower than 0, 0 is used then");
        m_excitation = 0;
    }
    else
        m_excitation = val;
}
void s2mMuscleStateDynamics::setActivation(const double &val) {
    m_previousActivation = m_activation;

    if (val<0){
        s2mError::s2mWarning(0, "Activation can't be lower than 0, 0 is used then");
        m_activation = 0;
    }
    else
        m_activation = val;
}

double s2mMuscleStateDynamics::excitationNorm(const s2mMuscleState &max) {
    s2mError::s2mWarning(m_excitation<max.excitation(), "Excitation is higher than maximal excitation.");
    m_excitationNorm = m_excitation / max.excitation();
    
    return m_excitationNorm;
}

double s2mMuscleStateDynamics::excitationNorm() const
{
    return m_excitationNorm;
}

void s2mMuscleStateDynamics::setExcitationNorm(double val)
{
    m_excitationNorm = val;
}

double s2mMuscleStateDynamics::previousActivation() const
{
    return m_previousActivation;
}

double s2mMuscleStateDynamics::previousExcitation() const
{
    return m_previousExcitation;
}
