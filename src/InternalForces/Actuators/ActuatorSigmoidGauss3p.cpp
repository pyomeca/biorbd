#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorSigmoidGauss3p.h"

#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

internal_forces::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p()
    : internal_forces::actuator::Actuator(),
      m_theta(std::make_shared<utils::Scalar>(0)),
      m_lambda(std::make_shared<utils::Scalar>(0)),
      m_offset(std::make_shared<utils::Scalar>(0)),
      m_r(std::make_shared<utils::Scalar>(0)),
      m_qopt(std::make_shared<utils::Scalar>(0)) {
  setType();
}

internal_forces::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    const internal_forces::actuator::ActuatorSigmoidGauss3p& other)
    : internal_forces::actuator::Actuator(other),
      m_theta(other.m_theta),
      m_lambda(other.m_lambda),
      m_offset(other.m_offset),
      m_r(other.m_r),
      m_qopt(other.m_qopt) {}

internal_forces::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const utils::Scalar& theta,
    const utils::Scalar& lambda,
    const utils::Scalar& offset,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    size_t dofIdx)
    : internal_forces::actuator::Actuator(direction, dofIdx),
      m_theta(std::make_shared<utils::Scalar>(theta)),
      m_lambda(std::make_shared<utils::Scalar>(lambda)),
      m_offset(std::make_shared<utils::Scalar>(offset)),
      m_r(std::make_shared<utils::Scalar>(r)),
      m_qopt(std::make_shared<utils::Scalar>(qopt)) {
  setType();
}

internal_forces::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const utils::Scalar& theta,
    const utils::Scalar& lambda,
    const utils::Scalar& offset,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    size_t dofIdx,
    const utils::String& jointName)
    : internal_forces::actuator::Actuator(direction, dofIdx, jointName),
      m_theta(std::make_shared<utils::Scalar>(theta)),
      m_lambda(std::make_shared<utils::Scalar>(lambda)),
      m_offset(std::make_shared<utils::Scalar>(offset)),
      m_r(std::make_shared<utils::Scalar>(r)),
      m_qopt(std::make_shared<utils::Scalar>(qopt)) {
  setType();
}

internal_forces::actuator::ActuatorSigmoidGauss3p::~ActuatorSigmoidGauss3p() {}

internal_forces::actuator::ActuatorSigmoidGauss3p
internal_forces::actuator::ActuatorSigmoidGauss3p::DeepCopy() const {
  internal_forces::actuator::ActuatorSigmoidGauss3p copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::actuator::ActuatorSigmoidGauss3p::DeepCopy(
    const internal_forces::actuator::ActuatorSigmoidGauss3p& other) {
  internal_forces::actuator::Actuator::DeepCopy(other);
  *m_theta = *other.m_theta;
  *m_lambda = *other.m_lambda;
  *m_offset = *other.m_offset;
  *m_r = *other.m_r;
  *m_qopt = *other.m_qopt;
}

utils::Scalar internal_forces::actuator::ActuatorSigmoidGauss3p::torqueMax() {
  utils::Error::raise(
      "torqueMax for ActuatorSigmoidGauss3p must be called with Q and Qdot");
}

utils::Scalar internal_forces::actuator::ActuatorSigmoidGauss3p::torqueMax(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot) {
  utils::Scalar pos(Q[static_cast<unsigned int>(*m_dofIdx)] * 180 / M_PI);
  utils::Scalar speed(Qdot[static_cast<unsigned int>(*m_dofIdx)] * 180 / M_PI);

  // Getting Tmax of Gauss3p from Sigmoid
  utils::Scalar Tmax(*m_theta / (1 + exp(*m_lambda * speed)) + *m_offset);

  // Calculation of the max torque
  return Tmax * exp(-(*m_qopt - pos) * (*m_qopt - pos) / (2 * *m_r * *m_r));
}

void internal_forces::actuator::ActuatorSigmoidGauss3p::setType() {
  *m_type = internal_forces::actuator::TYPE::SIGMOIDGAUSS3P;
}
