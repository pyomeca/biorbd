#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/StateDynamics.h"

#include "InternalForces/Muscles/Characteristics.h"
#include "Utils/Error.h"
#include "Utils/String.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::StateDynamics::StateDynamics(
    const utils::Scalar& excitation,
    const utils::Scalar& activation)
    : internal_forces::muscles::State(excitation, activation),
      m_previousExcitation(std::make_shared<utils::Scalar>(0)),
      m_previousActivation(std::make_shared<utils::Scalar>(0)),
      m_activationDot(std::make_shared<utils::Scalar>(0)) {
  setType();
}

internal_forces::muscles::StateDynamics::StateDynamics(
    const internal_forces::muscles::State& other)
    : internal_forces::muscles::State(other) {
  const auto& state =
      dynamic_cast<const internal_forces::muscles::StateDynamics&>(other);
  m_previousExcitation = state.m_previousExcitation;
  m_previousActivation = state.m_previousActivation;
  m_activationDot = state.m_activationDot;
}

internal_forces::muscles::StateDynamics::~StateDynamics() {
  // dtor
}

internal_forces::muscles::StateDynamics
internal_forces::muscles::StateDynamics::DeepCopy() const {
  internal_forces::muscles::StateDynamics copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::muscles::StateDynamics::DeepCopy(
    const internal_forces::muscles::StateDynamics& other) {
  internal_forces::muscles::State::DeepCopy(other);
  *m_excitationNorm = *other.m_excitationNorm;
  *m_previousExcitation = *other.m_previousExcitation;
  *m_previousActivation = *other.m_previousActivation;
  *m_activationDot = *other.m_activationDot;
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::timeDerivativeActivation(
    const internal_forces::muscles::State& emg,
    const internal_forces::muscles::Characteristics& characteristics,
    bool alreadyNormalized) {
  return timeDerivativeActivation(
      emg.excitation(), emg.activation(), characteristics, alreadyNormalized);
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::timeDerivativeActivation(
    const utils::Scalar& excitation,
    const utils::Scalar& activation,
    const internal_forces::muscles::Characteristics& characteristics,
    bool alreadyNormalized) {
  setExcitation(excitation);
  setActivation(activation);
  return timeDerivativeActivation(characteristics, alreadyNormalized);
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::timeDerivativeActivation(
    const internal_forces::muscles::Characteristics& characteristics,
    bool alreadyNormalized) {
  // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
  // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et
  // GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
#ifdef BIORBD_USE_CASADI_MATH
  *m_activation = IF_ELSE_NAMESPACE::if_else(
      IF_ELSE_NAMESPACE::lt(*m_activation, characteristics.minActivation()),
      characteristics.minActivation(),
      *m_activation);
  *m_excitation = IF_ELSE_NAMESPACE::if_else(
      IF_ELSE_NAMESPACE::lt(*m_excitation, characteristics.minActivation()),
      characteristics.minActivation(),
      *m_excitation);
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
    num = *m_excitation - *m_activation;  // numérateur
  } else {
    num = normalizeExcitation(characteristics.stateMax()) -
          *m_activation;  // numérateur
  }

  utils::Scalar denom;  // dénominateur
#ifdef BIORBD_USE_CASADI_MATH
  denom = IF_ELSE_NAMESPACE::if_else(
      IF_ELSE_NAMESPACE::gt(num, 0),
      characteristics.torqueActivation() * (0.5 + 1.5 * *m_activation),
      characteristics.torqueDeactivation() / (0.5 + 1.5 * *m_activation));
#else
  if (num > 0) {
    denom = characteristics.torqueActivation() * (0.5 + 1.5 * *m_activation);
  } else {
    denom = characteristics.torqueDeactivation() / (0.5 + 1.5 * *m_activation);
  }
#endif
  *m_activationDot = num / denom;
  return *m_activationDot;
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::timeDerivativeActivation() {
  return *m_activationDot;
}

void internal_forces::muscles::StateDynamics::setExcitation(
    const utils::Scalar& val,
    bool turnOffWarnings) {
  *m_previousExcitation = *m_excitation;
  internal_forces::muscles::State::setExcitation(val, turnOffWarnings);
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::previousExcitation() const {
  return *m_previousExcitation;
}

void internal_forces::muscles::StateDynamics::setActivation(
    const utils::Scalar& val,
    bool turnOffWarnings) {
  *m_previousActivation = *m_activation;
  internal_forces::muscles::State::setActivation(val, turnOffWarnings);
}

const utils::Scalar&
internal_forces::muscles::StateDynamics::previousActivation() const {
  return *m_previousActivation;
}

void internal_forces::muscles::StateDynamics::setType() {
  *m_stateType = internal_forces::muscles::STATE_TYPE::DYNAMIC;
}
