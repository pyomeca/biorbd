#define BIORBD_API_EXPORTS
#include "Utils/Quaternion.h"

#include <Eigen/Dense>
#include <rbdl/Quaternion.h>
#include "Utils/Node3d.h"
#include "Utils/Vector.h"

biorbd::utils::Quaternion::Quaternion (double kStabilizer) :
    RigidBodyDynamics::Math::Quaternion(),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion (const biorbd::utils::Vector &vec, double kStabilizer) :
    RigidBodyDynamics::Math::Quaternion(vec),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion (
        double w,
        double x,
        double y,
        double z,
        double kStabilizer) :
    RigidBodyDynamics::Math::Quaternion(x, y, z, w),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion (
        double w,
        const biorbd::utils::Node3d &vec3,
        double kStabilizer) :
    RigidBodyDynamics::Math::Quaternion(vec3(0), vec3(1), vec3(2), w),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion& biorbd::utils::Quaternion::operator=(
        const Eigen::Vector4d& other)
{
    if (this==&other) // check for self-assigment
        return *this;

    *this = other;
    return *this;
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
        biorbd::utils::Quaternion& other) const
{
    return biorbd::utils::Quaternion(this->RigidBodyDynamics::Math::Quaternion::operator*(other));
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator+(
        const biorbd::utils::Quaternion& other) const
{
	return biorbd::utils::Quaternion(this->RigidBodyDynamics::Math::Quaternion::operator+(other));
}

double biorbd::utils::Quaternion::w() const
{
    return (*this)(3);
}
double biorbd::utils::Quaternion::x() const
{
    return (*this)(0);
}
double biorbd::utils::Quaternion::y() const
{
    return (*this)(1);
}
double biorbd::utils::Quaternion::z() const
{
    return (*this)(2);
}

void biorbd::utils::Quaternion::derivate(
        const biorbd::utils::Vector &w)
{
    // Création du quaternion de "préproduit vectoriel"
    double qw = (*this)(3);
    double qx = (*this)(0);
    double qy = (*this)(1);
    double qz = (*this)(2);
    Eigen::Matrix4d Q;
    Q <<    qw, -qx, -qy, -qz,
            qx,  qw, -qz,  qy,
            qy,  qz,  qw, -qx,
            qz, -qy,  qx,  qw;

    // Ajout du paramètre de stabilisation
    Eigen::Vector4d w_tp (m_Kstab*w.norm()*(1-this->norm()), w(0), w(1), w(2));
    biorbd::utils::Quaternion quatDot(0.5 * Q * w_tp);
    biorbd::utils::Quaternion quatDot_tp(Eigen::Vector4d(quatDot(1), quatDot(2), quatDot(3), quatDot(0)));
    *this =  quatDot_tp; // Le quaternion est tourné à cause de la facon dont est faite la multiplication
}
