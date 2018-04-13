#include "../include/s2mJointIntraBone.h"

s2mJointIntraBone::s2mJointIntraBone() :
    Joint() {
    setType();
    //ctor
}
s2mJointIntraBone::s2mJointIntraBone(RigidBodyDynamics::JointType joint_type) :
            Joint(joint_type){
    setType();
}
s2mJointIntraBone::s2mJointIntraBone(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis) :
            Joint(joint_type, axis){
    setType();
}

s2mJointIntraBone::~s2mJointIntraBone()
{
    //dtor
}
