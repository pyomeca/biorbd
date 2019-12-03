#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque() {}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::GeneralizedTorque &other) :
    biorbd::utils::Vector(other)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(unsigned int nTorque) :
    biorbd::utils::Vector(nTorque)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbGeneralizedTorque())
{

}
