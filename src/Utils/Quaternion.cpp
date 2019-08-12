#define BIORBD_API_EXPORTS
#include "Utils/Quaternion.h"

#include <Eigen/Dense>
#include <rbdl/Quaternion.h>

biorbd::utils::Quaternion::Quaternion () : RigidBodyDynamics::Math::Quaternion(),
    m_Kstab(100)
{

}

biorbd::utils::Quaternion::Quaternion (const Eigen::Vector4d &vec4) :
    RigidBodyDynamics::Math::Quaternion(vec4),
    m_Kstab(100)
{

}

biorbd::utils::Quaternion::Quaternion (
        double x,
        double y,
        double z,
        double w) :
    RigidBodyDynamics::Math::Quaternion(x, y, z, w),
    m_Kstab(100)
{

}

biorbd::utils::Quaternion::Quaternion (
        const Eigen::Vector3d &vec4,
        double w) :
    RigidBodyDynamics::Math::Quaternion(vec4(0), vec4(1), vec4(2), w),
    m_Kstab(100)
{

}

biorbd::utils::Quaternion& biorbd::utils::Quaternion::operator=(const Eigen::Vector4d& vec4){
    if (this==&vec4) // check for self-assigment
        return *this;

    *this = vec4;
    return *this;
}


double biorbd::utils::Quaternion::w() const{
    return (*this)(3);
}
double biorbd::utils::Quaternion::x() const{
    return (*this)(0);
}
double biorbd::utils::Quaternion::y() const{
    return (*this)(1);
}
double biorbd::utils::Quaternion::z() const{
    return (*this)(2);
}

void biorbd::utils::Quaternion::derivate(const Eigen::VectorXd &w){

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
