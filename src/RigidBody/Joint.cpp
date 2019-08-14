#define BIORBD_API_EXPORTS
#include "RigidBody/Joint.h"

s2mJoint::s2mJoint() :
    Joint() {
    setType();
    //ctor
}
s2mJoint::s2mJoint(RigidBodyDynamics::JointType joint_type) :
            Joint(joint_type){
    setType();
}
s2mJoint::s2mJoint(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis) :
            Joint(joint_type, axis){
    setType();
}

s2mJoint::~s2mJoint()
{
    //dtor
}

const biorbd::utils::String &s2mJoint::type() const
{
    return m_type;
}

void s2mJoint::setType()
{
    m_type = "IntraBone";
}
