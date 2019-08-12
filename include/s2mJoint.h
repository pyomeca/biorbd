#ifndef S2M_JOINT_H
#define S2M_JOINT_H

#include <rbdl/Joint.h>
#include "biorbdConfig.h"
#include "Utils/String.h"

class BIORBD_API s2mJoint : public RigidBodyDynamics::Joint
{
    public:
        s2mJoint();
        s2mJoint(RigidBodyDynamics::JointType joint_type);
        s2mJoint(
                RigidBodyDynamics::JointType joint_type,
                RigidBodyDynamics::Math::Vector3d axis);
        virtual ~s2mJoint();

        const biorbd::utils::String& type() const;
    protected:
        virtual void setType ();
        biorbd::utils::String m_type;

};

#endif // S2M_JOINT_H
