#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedVelocity.h"

#include "RigidBody/Joints.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::GeneralizedVelocity::GeneralizedVelocity()
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    unsigned int nbQdot) :
    biorbd::utils::Vector(nbQdot)
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQdot())
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const rigidbody::GeneralizedVelocity &Q) :
    biorbd::utils::Vector(Q)
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif


rigidbody::GeneralizedVelocity::~GeneralizedVelocity()
{

}

void rigidbody::GeneralizedVelocity::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedVelocity::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void rigidbody::GeneralizedVelocity::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
