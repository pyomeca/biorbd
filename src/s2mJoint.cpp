#define BIORBD_API_EXPORTS
#include "../include/s2mJoint.h"


s2mJoint::s2mJoint() :
    Joint()
{

}

s2mJoint::s2mJoint(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis) :
    Joint(joint_type, axis)
{

}

s2mJoint::~s2mJoint()
{
    //dtor
}

const s2mString &s2mJoint::type() const {
    return m_type;
}
