#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamics_DeGroote_DeGroote.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::StateDynamics_DeGroote_DeGroote::StateDynamics_DeGroote_DeGroote(
        const biorbd::utils::Scalar& excitation,
        const biorbd::utils::Scalar& activation) :
    biorbd::muscles::State(excitation,activation),
    m_previousExcitation(std::make_shared<biorbd::utils::Scalar>(0)),
    m_previousActivation(std::make_shared<biorbd::utils::Scalar>(0)),
    m_activationDot(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::muscles::StateDynamics_DeGroote::StateDynamics_DeGroote(
        const biorbd::muscles::StateDynamics_DeGroote &other) :
    biorbd::muscles::State(other),
    m_previousExcitation(other.m_previousExcitation),
    m_previousActivation(other.m_previousActivation),
    m_activationDot(other.m_activationDot)
{

}

biorbd::muscles::StateDynamics_DeGroote::~StateDynamics_DeGroote()
{
    //dtor
}

biorbd::muscles::StateDynamics_DeGroote biorbd::muscles::StateDynamics_DeGroote::DeepCopy() const
{
    biorbd::muscles::StateDynamics_DeGroote copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamics_DeGroote::DeepCopy(const biorbd::muscles::StateDynamics_DeGroote &other)
{
    biorbd::muscles::State::DeepCopy(other);
    *m_excitationNorm = *other.m_excitationNorm;
    *m_previousExcitation = *other.m_previousExcitation;
    *m_previousActivation = *other.m_previousActivation;
    *m_activationDot = *other.m_activationDot;
}


const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::timeDerivativeActivation(
        const biorbd::muscles::StateDynamics_DeGroote& emg,
        const biorbd::muscles::Characteristics& characteristics,
        bool alreadyNormalized){
    return timeDerivativeActivation(emg.excitation(), emg.activation(), characteristics, alreadyNormalized);
}


const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::timeDerivativeActivation(
        const biorbd::utils::Scalar& excitation,
        const biorbd::utils::Scalar& activation,
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(characteristics, alreadyNormalized);
}

const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::timeDerivativeActivation(
        const biorbd::muscles::Characteristics &characteristics,
        bool alreadyNormalized){
    // Implémentation de la fonction DeGroote https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5043004/

    // Get activation and excitation
#ifdef BIORBD_USE_CASADI_MATH
    *m_activation = casadi::MX::if_else(
                casadi::MX::lt(*m_activation, characteristics.minActivation()),
                characteristics.minActivation(), *m_activation);
    *m_excitation = casadi::MX::if_else(
                casadi::MX::lt(*m_excitation, characteristics.minActivation()),
                characteristics.minActivation(), *m_excitation);
#else
    if (*m_activation < characteristics.minActivation())
        *m_activation = characteristics.minActivation();

    if (*m_excitation < characteristics.minActivation())
        *m_excitation = characteristics.minActivation();
#endif

// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    biorbd::utils::Scalar diff; //(e - a)
    biorbd::utils::Scalar f;    //activation dynamics

    if (alreadyNormalized)
        diff = *m_excitation - *m_activation;
    else
        diff = normalizeExcitation(characteristics.stateMax())- *m_activation;
    f = 0.5 * tanh(0.1*diff);
    
    biorbd::utils::Scalar denom_activation;   // dénominateur for activation
    biorbd::utils::Scalar denom_deactivation; // dénominateur for deactivation

    denom_activation = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
    denom_deactivation = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);

//#ifdef BIORBD_USE_CASADI_MATH
//    denom = casadi::MX::if_else(
//                casadi::MX::gt(num, 0),
//                characteristics.torqueActivation()   * (0.5+1.5* *m_activation),
//                characteristics.torqueDeactivation() / (0.5+1.5* *m_activation));
//#else
//    if (num>0)
//        denom = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
//    else
//        denom = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);
//#endif

    *m_activationDot = (((f+0.5)/denom_activation)+((-f + 0.5)/denom_deactivation))*diff;

    return *m_activationDot;
}


const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::timeDerivativeActivation()
{
    return *m_activationDot;
}

void biorbd::muscles::StateDynamics_DeGroote::setExcitation(
        const biorbd::utils::Scalar& val,
        bool turnOffWarnings) {
    *m_previousExcitation = *m_excitation;
    biorbd::muscles::State::setExcitation(val, turnOffWarnings);
}

const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::previousExcitation() const
{
    return *m_previousExcitation;
}

void biorbd::muscles::StateDynamics_DeGroote::setActivation(
        const biorbd::utils::Scalar& val,
        bool turnOffWarnings) {
    *m_previousActivation = *m_activation;
    biorbd::muscles::State::setActivation(val, turnOffWarnings);
}

const biorbd::utils::Scalar& biorbd::muscles::StateDynamics_DeGroote::previousActivation() const
{
    return *m_previousActivation;
}

void biorbd::muscles::StateDynamics_DeGroote::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::DYNAMIC;
}
