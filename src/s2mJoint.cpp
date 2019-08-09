#define BIORBD_API_EXPORTS
#include "s2mJoint.h"

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

const s2mString &s2mJoint::type() const
{
    return m_type;
}
