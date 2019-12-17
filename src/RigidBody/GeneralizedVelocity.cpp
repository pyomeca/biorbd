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
