#ifndef BIORBD_UTILS_QUATERNION_H
#define BIORBD_UTILS_QUATERNION_H

#include <memory>
#include <Eigen/Dense>
#include "Utils/Node3d.h"

#include "biorbdConfig.h"
namespace biorbd {
namespace utils {
class Node3d;
class Vector;
class RotoTrans;
class String;

// The definition for conversions are taken from https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
class BIORBD_API Quaternion : public Eigen::Vector4d
{
public:
    Quaternion (double kStabilizer = 1);
    Quaternion(const biorbd::utils::Quaternion &other);
    Quaternion (
            double w,
            double x,
            double y,
            double z,
            double kStabilizer = 1);
    Quaternion (
        double w,
        const biorbd::utils::Node3d &vec3, 
        double kStabilizer = 1);
    Quaternion (
        const biorbd::utils::Vector &vec,
        double kStabilizer = 1);

    double w() const;
    double x() const;
    double y() const;
    double z() const;

    void setKStab(double newKStab);
    double kStab() const;

    template<typename OtherDerived>
    biorbd::utils::Quaternion& operator=(
            const Eigen::MatrixBase <OtherDerived>& other){
        this->Eigen::Vector4d::operator=(other);
        // I don't understand why the next line doesn't SegFault...
        this->m_Kstab = static_cast<biorbd::utils::Quaternion>(other).m_Kstab;
        return *this;
    }
    biorbd::utils::Quaternion operator*(
            const biorbd::utils::Quaternion& other) const;
    biorbd::utils::Quaternion operator*(
            double s) const;
    biorbd::utils::Quaternion operator*(
            float other) const;
    biorbd::utils::Quaternion operator+(
            const biorbd::utils::Quaternion& other) const;
    biorbd::utils::Quaternion operator-(
            const biorbd::utils::Quaternion& other) const;

    static biorbd::utils::Quaternion fromGLRotate (
            double angle,
            double x,
            double y,
            double z,
            double kStab = 1);

    static biorbd::utils::Quaternion fromAxisAngle (
            double angle_rad,
            const biorbd::utils::Node3d &axis,
            double kStab = 1);

    static biorbd::utils::Quaternion fromMatrix (
            const biorbd::utils::RotoTrans &mat,
            double kStab = 1);

    static biorbd::utils::Quaternion fromMatrix (
            const Eigen::Matrix3d &mat,
            double kStab = 1);

    static biorbd::utils::Quaternion fromZYXAngles (
            const biorbd::utils::Node3d &zyx_angles,
            double kStab = 1);

    static biorbd::utils::Quaternion fromYXZAngles (
            const biorbd::utils::Node3d &yxz_angles,
            double kStab = 1);

    static biorbd::utils::Quaternion fromXYZAngles (
            const biorbd::utils::Node3d &xyz_angles,
            double kStab = 1);

    biorbd::utils::RotoTrans toMatrix() const;

    biorbd::utils::Quaternion slerp (
            double alpha,
            const Quaternion &quat) const;

    biorbd::utils::Quaternion conjugate() const;

    biorbd::utils::Quaternion timeStep (
            const Eigen::Vector3d &omega,
            double dt);

    biorbd::utils::Node3d rotate (const biorbd::utils::Node3d &vec) const;

    /** \brief Converts a 3d angular velocity vector into a 4d derivative of the
    * components of the quaternion.
    *
    * \param omega the angular velocity.
    *
    * \return a 4d vector containing the derivatives of the 4 components of the
    * quaternion corresponding to omega.
    *
    */
    biorbd::utils::Quaternion omegaToQDot(const biorbd::utils::Node3d& omega) const;

    /** \brief Converts a 3d velocity vector expressed in terms of euler angles rate 
    * into the 3d angular velocity vector expressed in the fixed parent frame.
    * See # https://davidbrown3.github.io/2017-07-25/EulerAngles/ for correct equations.
    *
    * \param eulerDot the Euler angle rates.
    * \param euler the Euler angles.
    * \param seq   the Euler angles sequence.
    * 
    * \return omega, the angular velocity.
    *
    */
    biorbd::utils::Node3d  eulerDotToOmega(
            const biorbd::utils::Node3d &eulerDot, 
            const biorbd::utils::Node3d &euler,
            const biorbd::utils::String& seq);

    void derivate(const biorbd::utils::Vector &w);

protected:
    double m_Kstab; // Facteur de stabilisation pour la derivation

};

}}

#endif // BIORBD_UTILS_QUATERNION_H
