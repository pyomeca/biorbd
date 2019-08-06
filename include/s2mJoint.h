#ifndef S2MJOINT_H
#define S2MJOINT_H
    #include "biorbdConfig.h"
    #include "s2mString.h"
    #include <rbdl/rbdl.h>
    #include "s2mBone.h"

class BIORBD_API s2mJoint : public RigidBodyDynamics::Joint
{
    public:
        s2mJoint();
        s2mJoint(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis);
        virtual ~s2mJoint();

        // Set and Get
        const s2mString& type() const;
    protected:
        virtual void setType (){m_type = "Static";}
        s2mString m_type;
    private:
};

#endif // S2MJOINT_H
