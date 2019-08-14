#define BIORBD_API_EXPORTS
#include "RigidBody/Joint.h"

biorbd::rigidbody::Joint::Joint() :
    RigidBodyDynamics::Joint()
{
    setType();
    //ctor
}
biorbd::rigidbody::Joint::Joint(RigidBodyDynamics::JointType joint_type) :
    RigidBodyDynamics::Joint(joint_type)
{
    setType();
}
biorbd::rigidbody::Joint::Joint(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis) :
    RigidBodyDynamics::Joint(joint_type, axis){
    setType();
}

biorbd::rigidbody::Joint::~Joint()
{
    //dtor
}

const biorbd::utils::String &biorbd::rigidbody::Joint::type() const
{
    return m_type;
}

void biorbd::rigidbody::Joint::setType()
{
    m_type = "IntraBone";
}
