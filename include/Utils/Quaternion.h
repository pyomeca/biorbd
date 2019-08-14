#ifndef BIORBD_UTILS_QUATERNION_H
#define BIORBD_UTILS_QUATERNION_H

#include <rbdl/rbdl_math.h>
#include <rbdl/Quaternion.h>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {

class BIORBD_API Quaternion : public RigidBodyDynamics::Math::Quaternion
{
public:
    Quaternion();
    Quaternion (const Eigen::Vector4d &vec4);
    Quaternion (
            const Eigen::Vector3d &vec4,
            double w);
    Quaternion (
            double x,
            double y,
            double z,
            double w);

    Quaternion& operator=(const Eigen::Vector4d& vec4);
    double w() const;
    double x() const;
    double y() const;
    double z() const;

    void derivate(const Eigen::VectorXd &w);
protected:
    double m_Kstab; // Facteur de stabilisation lors de la derivation

};

}}

#endif // BIORBD_UTILS_QUATERNION_H
