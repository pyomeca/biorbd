#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedAcceleration.h"

#include "RigidBody/Joints.h"

using namespace BIORBD_NAMESPACE;

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration() {}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(size_t nQddot)
    : utils::Vector(nQddot) {}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const rigidbody::Joints &j)
    : utils::Vector(j.nbQ()) {}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const rigidbody::GeneralizedAcceleration &Q)
    : utils::Vector(Q) {}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const RigidBodyDynamics::Math::VectorNd &v)
    : utils::Vector(v) {}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(const casadi::MX &v)
    : utils::Vector(v) {}

#endif

rigidbody::GeneralizedAcceleration::~GeneralizedAcceleration() {}

void rigidbody::GeneralizedAcceleration::operator=(const utils::Vector &other) {
  this->utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedAcceleration::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other) {
  this->utils::Vector::operator=(other);
}

void rigidbody::GeneralizedAcceleration::operator=(const casadi::MX &other) {
  this->utils::Vector::operator=(other);
}

#endif
