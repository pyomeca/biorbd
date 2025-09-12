#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/Actuator.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::actuator::Actuator::Actuator()
    : m_type(
          std::make_shared<internal_forces::actuator::TYPE>(
              internal_forces::actuator::TYPE::NO_TYPE)),
      m_direction(std::make_shared<int>(0)),
      m_jointName(std::make_shared<utils::String>("")),
      m_dofIdx(std::make_shared<size_t>(-1)) {}

internal_forces::actuator::Actuator::Actuator(
    const internal_forces::actuator::Actuator &other)
    : m_type(other.m_type),
      m_direction(other.m_direction),
      m_jointName(other.m_jointName),
      m_dofIdx(other.m_dofIdx) {}

internal_forces::actuator::Actuator::Actuator(int direction, size_t dofIdx)
    : m_type(
          std::make_shared<internal_forces::actuator::TYPE>(
              internal_forces::actuator::TYPE::NO_TYPE)),
      m_direction(std::make_shared<int>(direction)),
      m_jointName(std::make_shared<utils::String>("")),
      m_dofIdx(std::make_shared<size_t>(dofIdx)) {}

internal_forces::actuator::Actuator::Actuator(
    int direction,
    size_t dofIdx,
    const utils::String &jointName)
    : m_type(
          std::make_shared<internal_forces::actuator::TYPE>(
              internal_forces::actuator::TYPE::NO_TYPE)),
      m_direction(std::make_shared<int>(direction)),
      m_jointName(std::make_shared<utils::String>(jointName)),
      m_dofIdx(std::make_shared<size_t>(dofIdx)) {
  utils::Error::check(
      *m_direction == -1 || *m_direction == 1, "Direction should be -1 or 1");
}

internal_forces::actuator::Actuator::~Actuator() {}

void internal_forces::actuator::Actuator::DeepCopy(
    const internal_forces::actuator::Actuator &other) {
  *m_type = *other.m_type;
  *m_direction = *other.m_direction;
  *m_jointName = *other.m_jointName;
  *m_dofIdx = *other.m_dofIdx;
}

size_t internal_forces::actuator::Actuator::index() const { return *m_dofIdx; }

int internal_forces::actuator::Actuator::direction() const {
  return *m_direction;
}

internal_forces::actuator::TYPE internal_forces::actuator::Actuator::type()
    const {
  return *m_type;
}
