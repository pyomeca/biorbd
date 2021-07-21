#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

using namespace BIORBD_NAMESPACE;

rigidbody::GeneralizedTorque::GeneralizedTorque()
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    unsigned int nTorque) :
    utils::Vector(nTorque)
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const rigidbody::Joints &j) :
    utils::Vector(j.nbGeneralizedTorque())
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const rigidbody::GeneralizedTorque &other) :
    utils::Vector(other)
{

}

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const RigidBodyDynamics::Math::VectorNd &v) :
    utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedTorque::GeneralizedTorque(
    const casadi::MX &v) :
    utils::Vector(v)
{

}

#endif

void rigidbody::GeneralizedTorque::operator=(
    const utils::Vector &other)
{
    this->utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedTorque::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->utils::Vector::operator=(other);
}

void rigidbody::GeneralizedTorque::operator=(
    const casadi::MX &other)
{
    this->utils::Vector::operator=(other);
}

#endif
