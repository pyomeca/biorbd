#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenActiveOnlyType.h"

#include <math.h>

#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;
internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType()
    : internal_forces::muscles::HillThelenType() {
  setType();
}
internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics)
    : internal_forces::muscles::HillThelenType(
          name,
          geometry,
          characteristics) {
  setType();
}

internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg)
    : internal_forces::muscles::HillThelenType(
          name,
          geometry,
          characteristics,
          emg) {
  setType();
}

internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers)
    : internal_forces::muscles::HillThelenType(
          name,
          geometry,
          characteristics,
          pathModifiers) {
  setType();
}

internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    const internal_forces::muscles::State &emg)
    : internal_forces::muscles::HillThelenType(
          name,
          geometry,
          characteristics,
          pathModifiers,
          emg) {
  setType();
}

internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const internal_forces::muscles::Muscle &other)
    : internal_forces::muscles::HillThelenType(other) {}

internal_forces::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const std::shared_ptr<internal_forces::muscles::Muscle> other)
    : internal_forces::muscles::HillThelenType(other) {}

internal_forces::muscles::HillThelenActiveOnlyType
internal_forces::muscles::HillThelenActiveOnlyType::DeepCopy() const {
  internal_forces::muscles::HillThelenActiveOnlyType copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::muscles::HillThelenActiveOnlyType::DeepCopy(
    const internal_forces::muscles::HillThelenActiveOnlyType &other) {
  internal_forces::muscles::HillThelenType::DeepCopy(other);
}

void internal_forces::muscles::HillThelenActiveOnlyType::computeFlPE() {
  *m_FlPE = 0;
}

void internal_forces::muscles::HillThelenActiveOnlyType::computeDamping() {
  *m_damping = 0;
}

void internal_forces::muscles::HillThelenActiveOnlyType::setType() {
  *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE;
}
