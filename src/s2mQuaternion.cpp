#define BIORBD_API_EXPORTS
#include "../include/s2mQuaternion.h"


s2mQuaternion::s2mQuaternion () : RigidBodyDynamics::Math::Quaternion(),
    m_Kstab(100)
{

}

s2mQuaternion::s2mQuaternion (const Eigen::Vector4d &vec4) : RigidBodyDynamics::Math::Quaternion(vec4),
    m_Kstab(100)
{

}

s2mQuaternion::s2mQuaternion (double x, double y, double z, double w) : RigidBodyDynamics::Math::Quaternion(x, y, z, w),
    m_Kstab(100)
{

}

s2mQuaternion::s2mQuaternion (const Eigen::Vector3d &vec4, double w) : RigidBodyDynamics::Math::Quaternion(vec4(0), vec4(1), vec4(2), w),
    m_Kstab(100)
{

}

s2mQuaternion& s2mQuaternion::operator=(const Eigen::Vector4d& vec4){
    if (this==&vec4) // check for self-assigment
        return *this;

    *this = vec4;
    return *this;
}


double s2mQuaternion::w() const{
    return (*this)(3);
}
double s2mQuaternion::x() const{
    return (*this)(0);
}
double s2mQuaternion::y() const{
    return (*this)(1);
}
double s2mQuaternion::z() const{
    return (*this)(2);
}

void s2mQuaternion::derivate(const Eigen::VectorXd &w){

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
    s2mQuaternion quatDot(0.5 * Q * w_tp);
    s2mQuaternion quatDot_tp(Eigen::Vector4d(quatDot(1), quatDot(2), quatDot(3), quatDot(0)));
    *this =  quatDot_tp; // Le quaternion est tourné à cause de la facon dont est faite la multiplication
}
