#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque() {}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::GeneralizedTorque &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(unsigned int i) :
    biorbd::utils::Vector(i)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbGeneralizedTorque())
{

}
