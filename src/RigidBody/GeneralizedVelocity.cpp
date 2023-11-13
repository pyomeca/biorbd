#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedVelocity.h"

#include "RigidBody/Joints.h"

using namespace BIORBD_NAMESPACE;

rigidbody::GeneralizedVelocity::GeneralizedVelocity()
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    size_t nbQdot) :
    utils::Vector(nbQdot)
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const rigidbody::Joints &j) :
    utils::Vector(j.nbQdot())
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const rigidbody::GeneralizedVelocity &Q) :
    utils::Vector(Q)
{

}

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const RigidBodyDynamics::Math::VectorNd &v) :
    utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const casadi::MX &v) :
    utils::Vector(v)
{

}

#endif


rigidbody::GeneralizedVelocity::~GeneralizedVelocity()
{

}

void rigidbody::GeneralizedVelocity::operator=(
    const utils::Vector &other)
{
    this->utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void rigidbody::GeneralizedVelocity::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->utils::Vector::operator=(other);
}

void rigidbody::GeneralizedVelocity::operator=(
    const casadi::MX &other)
{
    this->utils::Vector::operator=(other);
}

#endif
