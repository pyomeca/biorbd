#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedAcceleration.h"

#include "RigidBody/Joints.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration()
{

}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    unsigned int nQddot) :
    biorbd::utils::Vector(nQddot)
{

}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQ())
{

}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const rigidbody::GeneralizedAcceleration &Q) :
    biorbd::utils::Vector(Q)
{

}

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif


rigidbody::GeneralizedAcceleration::~GeneralizedAcceleration()
{

}

void rigidbody::GeneralizedAcceleration::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedAcceleration::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void rigidbody::GeneralizedAcceleration::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
