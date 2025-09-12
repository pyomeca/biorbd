#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueModel.h"

#include "InternalForces/Muscles/FatigueDynamicStateXia.h"
#include "InternalForces/Muscles/Muscle.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::FatigueModel::FatigueModel(
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) {
  if (dynamicFatigueType ==
      internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE) {
    m_fatigueState = std::make_shared<internal_forces::muscles::FatigueState>();
  } else if (
      dynamicFatigueType ==
      internal_forces::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA) {
    m_fatigueState =
        std::make_shared<internal_forces::muscles::FatigueDynamicStateXia>();
  } else {
    utils::Error::raise("Wrong muscle fatigue type");
  }
}

internal_forces::muscles::FatigueModel::FatigueModel(
    const internal_forces::muscles::FatigueModel& other)
    : m_fatigueState(other.m_fatigueState) {}

internal_forces::muscles::FatigueModel::FatigueModel(
    const std::shared_ptr<internal_forces::muscles::FatigueModel> other)
    : m_fatigueState(other->m_fatigueState) {}

internal_forces::muscles::FatigueModel::~FatigueModel() {}

void internal_forces::muscles::FatigueModel::DeepCopy(
    const internal_forces::muscles::FatigueModel& other) {
  *m_fatigueState = other.m_fatigueState->DeepCopy();
}

#ifndef BIORBD_USE_CASADI_MATH
void internal_forces::muscles::FatigueModel::setFatigueState(
    const utils::Scalar& active,
    const utils::Scalar& fatigued,
    const utils::Scalar& resting) {
  m_fatigueState->setState(active, fatigued, resting);
}
#endif

internal_forces::muscles::FatigueState&
internal_forces::muscles::FatigueModel::fatigueState() {
  return *m_fatigueState;
}

const internal_forces::muscles::FatigueState&
internal_forces::muscles::FatigueModel::fatigueState() const {
  return *m_fatigueState;
}

void internal_forces::muscles::FatigueModel::computeTimeDerivativeState(
    const internal_forces::muscles::StateDynamics& emg) {
  if (std::dynamic_pointer_cast<internal_forces::muscles::FatigueDynamicState>(
          m_fatigueState)) {
    internal_forces::muscles::Muscle* muscle =
        dynamic_cast<internal_forces::muscles::Muscle*>(this);
    if (muscle) {
      std::static_pointer_cast<internal_forces::muscles::FatigueDynamicState>(
          m_fatigueState)
          ->timeDerivativeState(emg, muscle->characteristics());
    } else {
      utils::Error::raise(
          "internal_forces::muscles::FatigueModel should be a "
          "internal_forces::muscles::Muscle");
    }
  } else {
    utils::Error::raise("Type cannot be fatigued");
  }
}
