#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinates.h"

#include "RigidBody/Joints.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates()
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    unsigned int nbQ) :
    biorbd::utils::Vector(nbQ)
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQ())
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const rigidbody::GeneralizedCoordinates &Q) :
    biorbd::utils::Vector(Q)
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif

rigidbody::GeneralizedCoordinates::~GeneralizedCoordinates()
{

}

void rigidbody::GeneralizedCoordinates::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedCoordinates::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void rigidbody::GeneralizedCoordinates::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
