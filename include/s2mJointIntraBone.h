#ifndef S2MJOINTINTRABONE_H
#define S2MJOINTINTRABONE_H
    #include "biorbdConfig.h"
    #include "s2mString.h"
    #include <rbdl/rbdl.h>

class BIORBD_API s2mJointIntraBone : public RigidBodyDynamics::Joint
{
    public:
        s2mJointIntraBone();
        s2mJointIntraBone(RigidBodyDynamics::JointType joint_type);
        s2mJointIntraBone(RigidBodyDynamics::JointType joint_type, RigidBodyDynamics::Math::Vector3d axis);
        virtual ~s2mJointIntraBone();

        const s2mString& type() const;
    protected:
        virtual void setType (){m_type = "IntraBone";}
        s2mString m_type;
    private:
};


#endif // S2MJOINTINTRABONE_H
