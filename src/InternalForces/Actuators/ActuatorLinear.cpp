#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorLinear.h"

#include "RigidBody/GeneralizedCoordinates.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

internal_forces::actuator::ActuatorLinear::ActuatorLinear()
    : Actuator(),
      m_m(std::make_shared<utils::Scalar>(0)),
      m_b(std::make_shared<utils::Scalar>(0)) {
  setType();
}

internal_forces::actuator::ActuatorLinear::ActuatorLinear(
    const internal_forces::actuator::ActuatorLinear& other)
    : Actuator(other), m_m(other.m_m), m_b(other.m_b) {}

internal_forces::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    size_t dofIdx)
    : Actuator(direction, dofIdx),
      m_m(std::make_shared<utils::Scalar>(slope)),
      m_b(std::make_shared<utils::Scalar>(T0)) {
  setType();
}

internal_forces::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    size_t dofIdx,
    const utils::String& jointName)
    : Actuator(direction, dofIdx, jointName),
      m_m(std::make_shared<utils::Scalar>(slope)),
      m_b(std::make_shared<utils::Scalar>(T0)) {
  setType();
}

internal_forces::actuator::ActuatorLinear::~ActuatorLinear() {}

internal_forces::actuator::ActuatorLinear
internal_forces::actuator::ActuatorLinear::DeepCopy() const {
  internal_forces::actuator::ActuatorLinear copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::actuator::ActuatorLinear::DeepCopy(
    const internal_forces::actuator::ActuatorLinear& other) {
  internal_forces::actuator::Actuator::DeepCopy(other);
  *m_m = *other.m_m;
  *m_b = *other.m_b;
}

utils::Scalar internal_forces::actuator::ActuatorLinear::torqueMax() {
  utils::Error::raise(
      "torqueMax for ActuatorLinear must be called with Q and Qdot");
}

utils::Scalar internal_forces::actuator::ActuatorLinear::torqueMax(
    const rigidbody::GeneralizedCoordinates& Q) const {
  return (Q[static_cast<unsigned int>(*m_dofIdx)] * 180 / M_PI) * *m_m + *m_b;
}

void internal_forces::actuator::ActuatorLinear::setType() {
  *m_type = internal_forces::actuator::TYPE::LINEAR;
}
