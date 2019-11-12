#ifndef BIORBD_UTILS_QUATERNION_H
#define BIORBD_UTILS_QUATERNION_H

#include <memory>
#include <rbdl/rbdl_math.h>
#include <rbdl/Quaternion.h>
#include "biorbdConfig.h"
namespace biorbd {
namespace utils {
class Node3d;
class Vector;

class BIORBD_API Quaternion : public RigidBodyDynamics::Math::Quaternion
{
public:
    Quaternion(double kStabilizer = 1);
    Quaternion (
        const biorbd::utils::Vector &vec, 
        double kStabilizer = 1);
    Quaternion (
        double w,
        const biorbd::utils::Node3d &vec3, 
        double kStabilizer = 1);
    Quaternion (
            double w,
            double x,
            double y,
            double z,
            double kStabilizer = 1);

    biorbd::utils::Quaternion& operator=(const Eigen::Vector4d& other);
    biorbd::utils::Quaternion operator*(const biorbd::utils::Quaternion& other) const;
    biorbd::utils::Quaternion operator*(double other) const;
    biorbd::utils::Quaternion operator*(float other) const;
    biorbd::utils::Quaternion operator+(const biorbd::utils::Quaternion& other) const;
    biorbd::utils::Quaternion operator-(const biorbd::utils::Quaternion& other) const;

    double w() const;
    double x() const;
    double y() const;
    double z() const;
    double k_stab() const;

    void derivate(const biorbd::utils::Vector &w);
protected:
    double m_Kstab; // Facteur de stabilisation pour la derivation

};

}}

#endif // BIORBD_UTILS_QUATERNION_H
