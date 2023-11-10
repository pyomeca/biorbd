#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinates.h"

#include "RigidBody/Joints.h"

using namespace BIORBD_NAMESPACE;

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates()
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    size_t nbQ) :
    utils::Vector(nbQ)
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const rigidbody::Joints &j) :
    utils::Vector(j.nbQ())
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const rigidbody::GeneralizedCoordinates &Q) :
    utils::Vector(Q)
{

}

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const RigidBodyDynamics::Math::VectorNd &v) :
    utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const casadi::MX &v) :
    utils::Vector(v)
{

}

#endif

rigidbody::GeneralizedCoordinates::~GeneralizedCoordinates()
{

}

void rigidbody::GeneralizedCoordinates::operator=(
    const utils::Vector &other)
{
    this->utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedCoordinates::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->utils::Vector::operator=(other);
}

void rigidbody::GeneralizedCoordinates::operator=(
    const casadi::MX &other)
{
    this->utils::Vector::operator=(other);
}

#endif
