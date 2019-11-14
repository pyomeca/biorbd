#define BIORBD_API_EXPORTS
#include "Utils/Quaternion.h"

#include <Eigen/Dense>
#include "Utils/Node3d.h"
#include "Utils/Vector.h"

biorbd::utils::Quaternion::Quaternion (
        double kStabilizer) :
    Eigen::Vector4d (1, 0, 0, 0),
    m_Kstab(kStabilizer) {

}

biorbd::utils::Quaternion::Quaternion(const biorbd::utils::Quaternion &other) :
    Eigen::Vector4d (other),
    m_Kstab(other.m_Kstab) {

}

biorbd::utils::Quaternion::Quaternion (
        double w,
        double x,
        double y,
        double z,
        double kStabilizer) :
    Eigen::Vector4d(w, x, y, z),
    m_Kstab(kStabilizer) {

}

biorbd::utils::Quaternion::Quaternion (
        double w,
        const biorbd::utils::Node3d &vec3,
        double kStabilizer) :
    Eigen::Vector4d(w, vec3[0], vec3[1], vec3[2]),
    m_Kstab(kStabilizer) {

}

biorbd::utils::Quaternion::Quaternion(
        const biorbd::utils::Vector &vec,
        double kStabilizer):
    Eigen::Vector4d (vec),
    m_Kstab(kStabilizer) {

}

double biorbd::utils::Quaternion::w() const
{
    return (*this)(0);
}
double biorbd::utils::Quaternion::x() const
{
    return (*this)(1);
}
double biorbd::utils::Quaternion::y() const
{
    return (*this)(2);
}
double biorbd::utils::Quaternion::z() const
{
    return (*this)(3);
}

void biorbd::utils::Quaternion::setKStab(double newKStab)
{
    m_Kstab = newKStab;
}

double biorbd::utils::Quaternion::kStab() const
{
    return m_Kstab;
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
        const biorbd::utils::Quaternion& q) const
{
    return biorbd::utils::Quaternion (
        (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2] - (*this)[3] * q[3],
        (*this)[0] * q[1] + (*this)[1] * q[0] + (*this)[2] * q[3] - (*this)[3] * q[2],
        (*this)[0] * q[2] + (*this)[2] * q[0] + (*this)[3] * q[1] - (*this)[1] * q[3],
        (*this)[0] * q[3] + (*this)[3] * q[0] + (*this)[1] * q[2] - (*this)[2] * q[1],
            (this->m_Kstab + q.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
        double s) const
{
    return biorbd::utils::Quaternion (
                this->Eigen::Vector4d::operator*(s), this->m_Kstab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
        float s) const
{
    return biorbd::utils::Quaternion (
                this->Eigen::Vector4d::operator*(s), this->m_Kstab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator+(
        const biorbd::utils::Quaternion& other) const
{
    return biorbd::utils::Quaternion(this->Eigen::Vector4d::operator+(other),
                                     (this->m_Kstab + other.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator-(
        const biorbd::utils::Quaternion& other) const
{
    return biorbd::utils::Quaternion(this->Eigen::Vector4d::operator-(other),
                                     (this->m_Kstab + other.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromGLRotate(
        double angle,
        double x,
        double y,
        double z,
        double kStab) {
    double st = std::sin (angle * M_PI / 360.);
    return biorbd::utils::Quaternion (
                std::cos (angle * M_PI / 360.), st * x, st * y, st * z, kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromAxisAngle(
        double angle_rad,
        const biorbd::utils::Node3d &axis,
        double kStab) {
    double d = axis.norm();
    double s2 = std::sin (angle_rad * 0.5) / d;
    return biorbd::utils::Quaternion (
                std::cos(angle_rad * 0.5),
                axis[0] * s2, axis[1] * s2, axis[2] * s2, kStab
            );
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromMatrix(
        const Eigen::Matrix3d &mat,
        double kStab) {
    double w = std::sqrt (1. + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5;
    return Quaternion (
                w,
                (mat(1,2) - mat(2,1)) / (w * 4.),
                (mat(2,0) - mat(0,2)) / (w * 4.),
                (mat(0,1) - mat(1,0)) / (w * 4.),
                kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromZYXAngles(
        const Eigen::Vector3d &zyx_angles,
        double kStab) {
    return fromAxisAngle (zyx_angles[0], Eigen::Vector3d (0., 0., 1.), kStab)
            * fromAxisAngle (zyx_angles[1], Eigen::Vector3d (0., 1., 0.), kStab)
            * fromAxisAngle (zyx_angles[2], Eigen::Vector3d (1., 0., 0.), kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromYXZAngles(
        const Eigen::Vector3d &yxz_angles,
        double kStab) {
    return fromAxisAngle (yxz_angles[0], Eigen::Vector3d (0., 1., 0.), kStab)
            * fromAxisAngle (yxz_angles[1], Eigen::Vector3d (1., 0., 0.), kStab)
            * fromAxisAngle (yxz_angles[2], Eigen::Vector3d (0., 0., 1.), kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromXYZAngles(
        const Eigen::Vector3d &xyz_angles, double kStab) {
    return fromAxisAngle (xyz_angles[2], Eigen::Vector3d (0., 0., 1.), kStab)
            * fromAxisAngle (xyz_angles[1], Eigen::Vector3d (0., 1., 0.), kStab)
            * fromAxisAngle (xyz_angles[0], Eigen::Vector3d (1., 0., 0.), kStab);
}

Eigen::Matrix3d biorbd::utils::Quaternion::toMatrix() const {
    double w = (*this)[0];
    double x = (*this)[1];
    double y = (*this)[2];
    double z = (*this)[3];
    Eigen::Matrix3d out;
    out <<  1 - 2*y*y - 2*z*z,  2*x*y + 2*w*z,      2*x*z - 2*w*y,
            2*x*y - 2*w*z,      1 - 2*x*x - 2*z*z,  2*y*z + 2*w*x,
            2*x*z + 2*w*y,      2*y*z - 2*w*x,      1 - 2*x*x - 2*y*y;
    return out;
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::slerp(
        double alpha,
        const biorbd::utils::Quaternion &quat) const {
    // check whether one of the two has 0 length
    double s = std::sqrt (squaredNorm() * quat.squaredNorm());

    // division by 0.f is unhealthy!
    assert (s != 0.);

    double angle = acos (dot(quat) / s);
    if (angle == 0. || std::isnan(angle)) {
        return *this;
    }
    assert(!std::isnan(angle));

    double d = 1. / std::sin (angle);
    double p0 = std::sin ((1. - alpha) * angle);
    double p1 = std::sin (alpha * angle);

    if (dot (quat) < 0.) {
        return Quaternion( ((*this) * p0 - quat * p1) * d, this->m_Kstab);
    }
    return Quaternion( ((*this) * p0 + quat * p1) * d,
                       (this->m_Kstab + quat.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::conjugate() const {
    return biorbd::utils::Quaternion (
                (*this)[0],
                -(*this)[1],-(*this)[2],-(*this)[3],
                this->kStab()
            );
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::timeStep(
        const Eigen::Vector3d &omega,
        double dt) {
    double omega_norm = omega.norm();
    return fromAxisAngle (
                dt * omega_norm, omega / omega_norm, this->m_Kstab) * (*this);
}

biorbd::utils::Node3d biorbd::utils::Quaternion::rotate(
        const biorbd::utils::Node3d &vec) const {
    biorbd::utils::Quaternion vec_quat (0., vec);

    biorbd::utils::Quaternion res_quat(vec_quat * (*this));
    res_quat = conjugate() * res_quat;

    return biorbd::utils::Node3d(res_quat[1], res_quat[2], res_quat[3]);
}

#include <iostream>
Eigen::Vector4d biorbd::utils::Quaternion::omegaToQDot(
        const Eigen::Vector3d &omega) const {
    Eigen::MatrixXd m(4, 3);
    m(0, 0) = -(*this)[1];   m(0, 1) = -(*this)[2];   m(0, 2) = -(*this)[3];
    m(1, 0) =  (*this)[0];   m(1, 1) = -(*this)[3];   m(1, 2) =  (*this)[2];
    m(2, 0) =  (*this)[3];   m(2, 1) =  (*this)[0];   m(2, 2) = -(*this)[1];
    m(3, 0) = -(*this)[2];   m(3, 1) =  (*this)[1];   m(3, 2) =  (*this)[0];
    return Eigen::Vector4d(0.5 * m * omega);
}

void biorbd::utils::Quaternion::derivate(
        const biorbd::utils::Vector &w)
{
    // Création du quaternion de "préproduit vectoriel"
    double qw = (*this)(0);
    double qx = (*this)(1);
    double qy = (*this)(2);
    double qz = (*this)(3);
    Eigen::Matrix4d Q;
    Q <<    qw, -qx, -qy, -qz,
            qx,  qw, -qz,  qy,
            qy,  qz,  qw, -qx,
            qz, -qy,  qx,  qw;

    // Ajout du paramètre de stabilisation
    Eigen::Vector4d w_tp (m_Kstab*w.norm()*(1-this->norm()), w(0), w(1), w(2));
    double kStab = this->m_Kstab;
    *this = 0.5 * Q * w_tp; // Since it passes through a Mat*vec, it looses stab
    this->m_Kstab = kStab;
}
