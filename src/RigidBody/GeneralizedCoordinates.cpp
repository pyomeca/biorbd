#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinates.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates()
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(const biorbd::rigidbody::GeneralizedCoordinates &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(unsigned int nbDof) : biorbd::utils::Vector(nbDof)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbDof()){

}

biorbd::rigidbody::GeneralizedCoordinates::~GeneralizedCoordinates()
{

}
