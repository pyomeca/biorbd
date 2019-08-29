#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamics.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Caracteristics.h"

biorbd::muscles::StateDynamics::StateDynamics(
        double e,
        double a) :
    biorbd::muscles::State(e,a),
    m_excitationNorm(0),
    m_previousExcitation(0),
    m_previousActivation(0),
    m_activationDot(0)
{
}

biorbd::muscles::StateDynamics::~StateDynamics()
{
    //dtor
}


double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        const biorbd::muscles::StateDynamics& state,
        const biorbd::muscles::Caracteristics& caract,
        bool alreadyNormalized){
    return timeDerivativeActivation(state.excitation(), state.activation(), caract, alreadyNormalized);
}


double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        double excitation,
        double activation,
        const biorbd::muscles::Caracteristics &caract,
        bool alreadyNormalized){
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(caract, alreadyNormalized);
}

double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        const biorbd::muscles::Caracteristics &caract,
        bool alreadyNormalized){
    // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
    // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
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
        denom = caract.GeneralizedTorqueActivation()   * (0.5+1.5*m_activation);
    else
        denom = caract.GeneralizedTorqueDeactivation() / (0.5+1.5*m_activation);

    m_activationDot = num/denom;

	return m_activationDot;
}

double biorbd::muscles::StateDynamics::timeDerivativeActivation() {return m_activationDot;}

void biorbd::muscles::StateDynamics::setExcitation(double val) {
    m_previousExcitation = m_excitation;
    if (val<0){
        biorbd::utils::Error::warning(0, "Excitation can't be lower than 0, 0 is used then");
        m_excitation = 0;
    }
    else
        m_excitation = val;
}
void biorbd::muscles::StateDynamics::setActivation(double val) {
    m_previousActivation = m_activation;

    if (val<0){
        biorbd::utils::Error::warning(0, "Activation can't be lower than 0, 0 is used then");
        m_activation = 0;
    }
    else
        m_activation = val;
}

double biorbd::muscles::StateDynamics::excitationNorm(const biorbd::muscles::State &max) {
    biorbd::utils::Error::warning(m_excitation<max.excitation(), "Excitation is higher than maximal excitation.");
    m_excitationNorm = m_excitation / max.excitation();
    
    return m_excitationNorm;
}

double biorbd::muscles::StateDynamics::excitationNorm() const
{
    return m_excitationNorm;
}

void biorbd::muscles::StateDynamics::setExcitationNorm(double val)
{
    m_excitationNorm = val;
}

double biorbd::muscles::StateDynamics::previousActivation() const
{
    return m_previousActivation;
}

double biorbd::muscles::StateDynamics::previousExcitation() const
{
    return m_previousExcitation;
}

void biorbd::muscles::StateDynamics::setType()
{
    m_stateType = biorbd::muscles::STATE_TYPE::DYNAMIC;
}
