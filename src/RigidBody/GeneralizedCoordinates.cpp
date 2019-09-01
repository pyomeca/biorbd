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

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(unsigned int i) : biorbd::utils::Vector(i)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbDof()){

}

biorbd::rigidbody::GeneralizedCoordinates::~GeneralizedCoordinates()
{

}

biorbd::rigidbody::GeneralizedCoordinates biorbd::rigidbody::GeneralizedCoordinates::DeepCopy() const
{
    biorbd::rigidbody::GeneralizedCoordinates copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::GeneralizedCoordinates::DeepCopy(const biorbd::rigidbody::GeneralizedCoordinates &other)
{
    biorbd::utils::Vector::DeepCopy(other);
}
