#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorque.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;
internal_forces::passive_torques::PassiveTorque::PassiveTorque()
    : m_type(
          std::make_shared<internal_forces::passive_torques::TORQUE_TYPE>(
              internal_forces::passive_torques::TORQUE_TYPE::NO_TORQUE_TYPE)),
      m_jointName(std::make_shared<utils::String>("")),
      m_dofIdx(std::make_shared<size_t>(-1)) {}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    const internal_forces::passive_torques::PassiveTorque &other)
    : m_type(other.m_type),
      m_jointName(other.m_jointName),
      m_dofIdx(other.m_dofIdx) {}
internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    const std::shared_ptr<internal_forces::passive_torques::PassiveTorque>
        other)
    : m_type(other->m_type),
      m_jointName(other->m_jointName),
      m_dofIdx(other->m_dofIdx) {}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(size_t dofIdx)
    : m_type(
          std::make_shared<internal_forces::passive_torques::TORQUE_TYPE>(
              internal_forces::passive_torques::TORQUE_TYPE::NO_TORQUE_TYPE)),
      m_jointName(std::make_shared<utils::String>("")),
      m_dofIdx(std::make_shared<size_t>(dofIdx)) {}

internal_forces::passive_torques::PassiveTorque::PassiveTorque(
    size_t dofIdx,
    const utils::String &jointName)
    : m_type(
          std::make_shared<internal_forces::passive_torques::TORQUE_TYPE>(
              internal_forces::passive_torques::TORQUE_TYPE::NO_TORQUE_TYPE)),
      m_jointName(std::make_shared<utils::String>(jointName)),
      m_dofIdx(std::make_shared<size_t>(dofIdx)) {}

internal_forces::passive_torques::PassiveTorque::~PassiveTorque() {}

void internal_forces::passive_torques::PassiveTorque::DeepCopy(
    const internal_forces::passive_torques::PassiveTorque &other) {
  *m_type = *other.m_type;
  *m_jointName = *other.m_jointName;
  *m_dofIdx = *other.m_dofIdx;
}

size_t internal_forces::passive_torques::PassiveTorque::index() const {
  return *m_dofIdx;
}

internal_forces::passive_torques::TORQUE_TYPE
internal_forces::passive_torques::PassiveTorque::type() const {
  return *m_type;
}
