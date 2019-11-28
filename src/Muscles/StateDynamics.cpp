#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamics.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::StateDynamics::StateDynamics(double excitation,
        double activation) :
    biorbd::muscles::State(excitation,activation),
    m_excitationNorm(std::make_shared<double>(0)),
    m_previousExcitation(std::make_shared<double>(0)),
    m_previousActivation(std::make_shared<double>(0)),
    m_activationDot(std::make_shared<double>(0))
{
    setType();
}

biorbd::muscles::StateDynamics::StateDynamics(
        const biorbd::muscles::StateDynamics &other) :
    biorbd::muscles::State(other),
    m_excitationNorm(other.m_excitationNorm),
    m_previousExcitation(other.m_previousExcitation),
    m_previousActivation(other.m_previousActivation),
    m_activationDot(other.m_activationDot)
{

}

biorbd::muscles::StateDynamics::~StateDynamics()
{
    //dtor
}

biorbd::muscles::StateDynamics biorbd::muscles::StateDynamics::DeepCopy() const
{
    biorbd::muscles::StateDynamics copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamics::DeepCopy(const biorbd::muscles::StateDynamics &other)
{
    biorbd::muscles::State::DeepCopy(other);
    *m_excitationNorm = *other.m_excitationNorm;
    *m_previousExcitation = *other.m_previousExcitation;
    *m_previousActivation = *other.m_previousActivation;
    *m_activationDot = *other.m_activationDot;
}


double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        const biorbd::muscles::StateDynamics& emg,
        const biorbd::muscles::Characteristics& characteristics,
        bool alreadyNormalized){
    return timeDerivativeActivation(emg.excitation(), emg.activation(), characteristics, alreadyNormalized);
}


double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        double excitation,
        double activation,
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(characteristics, alreadyNormalized);
}

double biorbd::muscles::StateDynamics::timeDerivativeActivation(
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
    // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
    if (*m_activation<characteristics.minActivation())
        *m_activation = characteristics.minActivation();

    if (*m_excitation<characteristics.minActivation())
        *m_excitation = characteristics.minActivation();



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    double num;
    if (alreadyNormalized)
        num = *m_excitation- *m_activation; // numérateur
    else
        num = excitationNorm(characteristics.stateMax())- *m_activation; // numérateur
    
    double denom; // dénominateur
    if (num>0)
        denom = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
    else
        denom = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);

    *m_activationDot = num/denom;

    return *m_activationDot;
}

double biorbd::muscles::StateDynamics::timeDerivativeActivation()
{
    return *m_activationDot;
}

void biorbd::muscles::StateDynamics::setExcitation(double val) {
    *m_previousExcitation = *m_excitation;
    if (val<0){
        biorbd::utils::Error::warning(0, "Excitation can't be lower than 0, 0 is used then");
        *m_excitation = 0;
    }
    else
        *m_excitation = val;
}
void biorbd::muscles::StateDynamics::setActivation(double val) {
    *m_previousActivation = *m_activation;

    if (val<0){
        biorbd::utils::Error::warning(0, "Activation can't be lower than 0, 0 is used then");
        *m_activation = 0;
    }
    else
        *m_activation = val;
}

double biorbd::muscles::StateDynamics::excitationNorm(const biorbd::muscles::State &max) {
    biorbd::utils::Error::warning(*m_excitation<max.excitation(), "Excitation is higher than maximal excitation.");
    *m_excitationNorm = *m_excitation / max.excitation();
    
    return *m_excitationNorm;
}

double biorbd::muscles::StateDynamics::excitationNorm() const
{
    return *m_excitationNorm;
}

void biorbd::muscles::StateDynamics::setExcitationNorm(double val)
{
    *m_excitationNorm = val;
}

double biorbd::muscles::StateDynamics::previousActivation() const
{
    return *m_previousActivation;
}

double biorbd::muscles::StateDynamics::previousExcitation() const
{
    return *m_previousExcitation;
}

void biorbd::muscles::StateDynamics::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::DYNAMIC;
}
