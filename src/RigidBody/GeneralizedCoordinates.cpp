#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinates.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates()
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(const GeneralizedCoordinates &Q):
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(const biorbd::utils::Vector &v) :
    biorbd::utils::Vector(v)
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

biorbd::rigidbody::GeneralizedCoordinates& biorbd::rigidbody::GeneralizedCoordinates::operator=(const biorbd::utils::Vector& vecX){
    if (this==&vecX) // check for self-assigment
        return *this;

    this->block(0,0,this->rows(),1) = vecX;

    return *this;
}

biorbd::rigidbody::GeneralizedCoordinates &biorbd::rigidbody::GeneralizedCoordinates::operator=(const Eigen::VectorXd &vecX)
{
    if (this==&vecX) // check for self-assigment
        return *this;

    this->block(0,0,this->rows(),1) = vecX;

    return *this;
}
