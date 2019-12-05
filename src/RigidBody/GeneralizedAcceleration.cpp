#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedAcceleration.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration()
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
        const biorbd::rigidbody::GeneralizedAcceleration &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
        unsigned int nQddot) :
    biorbd::utils::Vector(nQddot)
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
        const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQ()){

}

biorbd::rigidbody::GeneralizedAcceleration::~GeneralizedAcceleration()
{

}
