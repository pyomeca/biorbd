#ifndef BIORBD_RIGIDBODY_JOINT_H
#define BIORBD_RIGIDBODY_JOINT_H

#include <rbdl/Joint.h>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd {
namespace rigidbody {

class BIORBD_API Joint : public RigidBodyDynamics::Joint
{
    public:
        Joint();
        Joint(RigidBodyDynamics::JointType joint_type);
        Joint(
                RigidBodyDynamics::JointType joint_type,
                RigidBodyDynamics::Math::Vector3d axis);
        virtual ~Joint();

        const biorbd::utils::String& type() const;
    protected:
        virtual void setType ();
        biorbd::utils::String m_type;

};

}}

#endif // BIORBD_RIGIDBODY_JOINT_H
