#ifndef S2MQUATERNION_H
#define S2MQUATERNION_H
#include <rbdl.h>
#include "biorbdConfig.h"
    
class BIORBD_API s2mQuaternion : public RigidBodyDynamics::Math::Quaternion
{
    public:
        s2mQuaternion();
        s2mQuaternion (const Eigen::Vector4d &vec4);
        s2mQuaternion (const Eigen::Vector3d &vec4, double w);
        s2mQuaternion (double x, double y, double z, double w);
        s2mQuaternion& operator=(const Eigen::Vector4d& vec4);
        double w() const;
        double x() const;
        double y() const;
        double z() const;

        void derivate(const Eigen::VectorXd &w);
    protected:
        double m_Kstab; // Facteur de stabilisation lors de la derivation

private:
};
#endif // S2MQUATERNION_H
