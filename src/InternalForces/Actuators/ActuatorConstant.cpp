#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorConstant.h"

#include "RigidBody/GeneralizedCoordinates.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::actuator::ActuatorConstant::ActuatorConstant()
    : Actuator(), m_Tmax(std::make_shared<utils::Scalar>(0)) {
  setType();
}

internal_forces::actuator::ActuatorConstant::ActuatorConstant(
    const internal_forces::actuator::ActuatorConstant &other)
    : Actuator(other), m_Tmax(other.m_Tmax) {}

internal_forces::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar &Tmax,
    size_t dofIdx)
    : Actuator(direction, dofIdx),
      m_Tmax(std::make_shared<utils::Scalar>(Tmax)) {
  setType();
}

internal_forces::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar &Tmax,
    size_t dofIdx,
    const utils::String &jointName)
    : Actuator(direction, dofIdx, jointName),
      m_Tmax(std::make_shared<utils::Scalar>(Tmax)) {
  setType();
}

internal_forces::actuator::ActuatorConstant
internal_forces::actuator::ActuatorConstant::DeepCopy() const {
  internal_forces::actuator::ActuatorConstant copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::actuator::ActuatorConstant::DeepCopy(
    const internal_forces::actuator::ActuatorConstant &other) {
  internal_forces::actuator::Actuator::DeepCopy(other);
  *m_Tmax = *other.m_Tmax;
}

utils::Scalar internal_forces::actuator::ActuatorConstant::torqueMax() {
  return *m_Tmax;
}

void internal_forces::actuator::ActuatorConstant::setType() {
  *m_type = internal_forces::actuator::TYPE::CONSTANT;
}
