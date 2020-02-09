#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedVelocity.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity()
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
        const biorbd::rigidbody::GeneralizedVelocity &Q) :
    biorbd::utils::Vector(Q)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
        const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#endif

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
        unsigned int nbQdot) :
    biorbd::utils::Vector(nbQdot)
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
        const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQdot()){

}

biorbd::rigidbody::GeneralizedVelocity::~GeneralizedVelocity()
{

}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::rigidbody::GeneralizedVelocity::operator=(
        const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void biorbd::rigidbody::GeneralizedVelocity::operator=(
        const MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
