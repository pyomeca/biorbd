#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamics.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::StateDynamics::StateDynamics(
    const utils::Scalar& excitation,
    const utils::Scalar& activation) :
    muscles::State(excitation,activation),
    m_previousExcitation(std::make_shared<utils::Scalar>(0)),
    m_previousActivation(std::make_shared<utils::Scalar>(0)),
    m_activationDot(std::make_shared<utils::Scalar>(0))
{
    setType();
}

muscles::StateDynamics::StateDynamics(
    const muscles::State &other) :
    muscles::State(other)
{
    const auto& state = dynamic_cast<const muscles::StateDynamics&>(other);
    m_previousExcitation = state.m_previousExcitation;
    m_previousActivation = state.m_previousActivation;
    m_activationDot = state.m_activationDot;
}

muscles::StateDynamics::~StateDynamics()
{
    //dtor
}

muscles::StateDynamics muscles::StateDynamics::DeepCopy() const
{
    muscles::StateDynamics copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::StateDynamics::DeepCopy(const
        muscles::StateDynamics &other)
{
    muscles::State::DeepCopy(other);
    *m_excitationNorm = *other.m_excitationNorm;
    *m_previousExcitation = *other.m_previousExcitation;
    *m_previousActivation = *other.m_previousActivation;
    *m_activationDot = *other.m_activationDot;
}


const utils::Scalar&
muscles::StateDynamics::timeDerivativeActivation(
    const muscles::State& emg,
    const muscles::Characteristics& characteristics,
    bool alreadyNormalized)
{
    return timeDerivativeActivation(emg.excitation(), emg.activation(),
                                    characteristics, alreadyNormalized);
}


const utils::Scalar&
muscles::StateDynamics::timeDerivativeActivation(
    const utils::Scalar& excitation,
    const utils::Scalar& activation,
    const muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(characteristics, alreadyNormalized);
}

const utils::Scalar&
muscles::StateDynamics::timeDerivativeActivation(
    const muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
    // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
#ifdef BIORBD_USE_CASADI_MATH
    *m_activation = casadi::MX::if_else(
                        casadi::MX::lt(*m_activation, characteristics.minActivation()),
                        characteristics.minActivation(), *m_activation);
    *m_excitation = casadi::MX::if_else(
                        casadi::MX::lt(*m_excitation, characteristics.minActivation()),
                        characteristics.minActivation(), *m_excitation);
#else
    if (*m_activation < characteristics.minActivation()) {
        *m_activation = characteristics.minActivation();
    }

    if (*m_excitation < characteristics.minActivation()) {
        *m_excitation = characteristics.minActivation();
    }
#endif



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    utils::Scalar num;
    if (alreadyNormalized) {
        num = *m_excitation - *m_activation;    // numérateur
    } else {
        num = normalizeExcitation(characteristics.stateMax())-
              *m_activation;    // numérateur
    }

    utils::Scalar denom; // dénominateur
#ifdef BIORBD_USE_CASADI_MATH
    denom = casadi::MX::if_else(
                casadi::MX::gt(num, 0),
                characteristics.torqueActivation()   * (0.5+1.5* *m_activation),
                characteristics.torqueDeactivation() / (0.5+1.5* *m_activation));
#else
    if (num>0) {
        denom = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
    } else {
        denom = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);
    }
#endif
    *m_activationDot = num/denom;

    return *m_activationDot;
}

const utils::Scalar&
muscles::StateDynamics::timeDerivativeActivation()
{
    return *m_activationDot;
}

void muscles::StateDynamics::setExcitation(
    const utils::Scalar& val,
    bool turnOffWarnings)
{
    *m_previousExcitation = *m_excitation;
    muscles::State::setExcitation(val, turnOffWarnings);
}

const utils::Scalar&
muscles::StateDynamics::previousExcitation() const
{
    return *m_previousExcitation;
}

void muscles::StateDynamics::setActivation(
    const utils::Scalar& val,
    bool turnOffWarnings)
{
    *m_previousActivation = *m_activation;
    muscles::State::setActivation(val, turnOffWarnings);
}

const utils::Scalar&
muscles::StateDynamics::previousActivation() const
{
    return *m_previousActivation;
}

void muscles::StateDynamics::setType()
{
    *m_stateType = muscles::STATE_TYPE::DYNAMIC;
}
