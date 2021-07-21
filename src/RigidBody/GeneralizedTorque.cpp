#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::GeneralizedTorque::GeneralizedTorque()
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    unsigned int nTorque) :
    biorbd::utils::Vector(nTorque)
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbGeneralizedTorque())
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const rigidbody::GeneralizedTorque &other) :
    biorbd::utils::Vector(other)
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif

void rigidbody::GeneralizedTorque::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedTorque::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void rigidbody::GeneralizedTorque::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
