#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillDeGrooteActiveOnlyType.h"

#include <math.h>

#include "InternalForces/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;
internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType()
    : internal_forces::muscles::HillDeGrooteType() {
  setType();
}
internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(
        const utils::String &name,
        const internal_forces::muscles::MuscleGeometry &geometry,
        const internal_forces::muscles::Characteristics &characteristics)
    : internal_forces::muscles::HillDeGrooteType(
          name,
          geometry,
          characteristics) {
  setType();
}

internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(
        const utils::String &name,
        const internal_forces::muscles::MuscleGeometry &geometry,
        const internal_forces::muscles::Characteristics &characteristics,
        const internal_forces::muscles::State &emg)
    : internal_forces::muscles::HillDeGrooteType(
          name,
          geometry,
          characteristics,
          emg) {
  setType();
}

internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(
        const utils::String &name,
        const internal_forces::muscles::MuscleGeometry &geometry,
        const internal_forces::muscles::Characteristics &characteristics,
        const internal_forces::PathModifiers &pathModifiers)
    : internal_forces::muscles::HillDeGrooteType(
          name,
          geometry,
          characteristics,
          pathModifiers) {
  setType();
}

internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(
        const utils::String &name,
        const internal_forces::muscles::MuscleGeometry &geometry,
        const internal_forces::muscles::Characteristics &characteristics,
        const internal_forces::PathModifiers &pathModifiers,
        const internal_forces::muscles::State &emg)
    : internal_forces::muscles::HillDeGrooteType(
          name,
          geometry,
          characteristics,
          pathModifiers,
          emg) {
  setType();
}

internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(const internal_forces::muscles::Muscle &other)
    : internal_forces::muscles::HillDeGrooteType(other) {}

internal_forces::muscles::HillDeGrooteActiveOnlyType::
    HillDeGrooteActiveOnlyType(
        const std::shared_ptr<internal_forces::muscles::Muscle> other)
    : internal_forces::muscles::HillDeGrooteType(other) {}

internal_forces::muscles::HillDeGrooteActiveOnlyType
internal_forces::muscles::HillDeGrooteActiveOnlyType::DeepCopy() const {
  internal_forces::muscles::HillDeGrooteActiveOnlyType copy;
  copy.DeepCopy(*this);
  return copy;
}

void internal_forces::muscles::HillDeGrooteActiveOnlyType::DeepCopy(
    const internal_forces::muscles::HillDeGrooteActiveOnlyType &other) {
  internal_forces::muscles::HillDeGrooteType::DeepCopy(other);
}

void internal_forces::muscles::HillDeGrooteActiveOnlyType::computeFlPE() {
  *m_FlPE = 0;
}

void internal_forces::muscles::HillDeGrooteActiveOnlyType::computeDamping() {
  *m_damping = 0;
}

void internal_forces::muscles::HillDeGrooteActiveOnlyType::setType() {
  *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_ACTIVE;
}
