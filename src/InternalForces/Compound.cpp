#define BIORBD_API_EXPORTS
#include "InternalForces/Compound.h"

#include "InternalForces/PathModifiers.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

internal_forces::Compound::Compound()
    : m_name(std::make_shared<utils::String>("")),
      m_pathChanger(std::make_shared<internal_forces::PathModifiers>()),
      m_force(std::make_shared<utils::Scalar>(0)) {}

internal_forces::Compound::Compound(const utils::String &name)
    : m_name(std::make_shared<utils::String>(name)),
      m_pathChanger(std::make_shared<internal_forces::PathModifiers>()),
      m_force(std::make_shared<utils::Scalar>(0)) {}

internal_forces::Compound::Compound(
    const utils::String &name,
    const internal_forces::PathModifiers &pathModifiers)
    : m_name(std::make_shared<utils::String>(name)),
      m_pathChanger(
          std::make_shared<internal_forces::PathModifiers>(pathModifiers)),
      m_force(std::make_shared<utils::Scalar>(0)) {}

internal_forces::Compound::Compound(const internal_forces::Compound &other)
    : m_name(other.m_name),
      m_pathChanger(other.m_pathChanger),
      m_force(other.m_force) {}

internal_forces::Compound::Compound(
    std::shared_ptr<internal_forces::Compound> other)
    : m_name(other->m_name),
      m_pathChanger(other->m_pathChanger),
      m_force(other->m_force) {}

internal_forces::Compound::~Compound() {}

void internal_forces::Compound::DeepCopy(
    const internal_forces::Compound &other) {
  *m_name = *other.m_name;
  *m_pathChanger = other.m_pathChanger->DeepCopy();
  *m_force = *other.m_force;
}

const utils::String &internal_forces::Compound::name() const { return *m_name; }

void internal_forces::Compound::setName(const utils::String &name) {
  *m_name = name;
}

const internal_forces::PathModifiers &
internal_forces::Compound::pathModifier() {
  return *m_pathChanger;
}

void internal_forces::Compound::addPathObject(utils::Vector3d &wrap) {
  m_pathChanger->addPathChanger(wrap);
}

const utils::Scalar &internal_forces::Compound::force() { return *m_force; }
