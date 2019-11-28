#ifndef BIORBD_UTILS_QUATERNION_H
#define BIORBD_UTILS_QUATERNION_H

#include <memory>
#include <Eigen/Dense>
#include "Utils/Vector3d.h"

#include "biorbdConfig.h"
namespace biorbd {
namespace utils {
class Vector3d;
class Vector;
class RotoTrans;
class String;

// The definition for conversions are taken from https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
///
/// Class Quaternion
///
class BIORBD_API Quaternion : public Eigen::Vector4d
{
public:
    ///
    /// \brief Construct Quaternion
    /// \param kStabilizer Stabilizer (default: 1)
    ///
    Quaternion (double kStabilizer = 1);
    
    ///
    /// \brief Construct Quaternion from another quaternion
    /// \param other Other quaternion
    ///
    Quaternion(const biorbd::utils::Quaternion &other);
    ///
    /// \brief Construct Quaternion
    /// \param w W parameter of quaternion
    /// \param x X parameter of quaternion
    /// \param y Y parameter of quaternion
    /// \param z Z parameter of quaternion
    /// \param kStabilizer Stabilizer - Imaginary coefficient (default: 1)
    ///
    Quaternion (
            double w,
            double x,
            double y,
            double z,
            double kStabilizer = 1);
    
    ///
    /// \brief Construct Quaternion
    /// \param w W parameter of quaternion (scalar)
    /// \param vec3 3d vector
    /// \param kStabilizer Stabilizer - Imaginary coefficient (default:1)
    ///
    Quaternion (
        double w,
        const biorbd::utils::Vector3d &vec3, 
        double kStabilizer = 1);
    ///
    /// \brief Construct Quaternion
    /// \param vec vector
    /// \param kStabilizer Stabilizer - Imaginary coefficient (default : 1)
    Quaternion (
        const biorbd::utils::Vector &vec,
        double kStabilizer = 1);

    ///
    /// \brief Return the W parameter of a Quaternion
    /// \return The W parameter of a Quaternion
    ///
    double w() const;
    ///
    /// \brief Return the X parameter of a Quaternion
    /// \return The X parameter of a Quaternion
    ///
    double x() const;
    ///
    /// \brief Return the Y parameter of a Quaternion
    /// \return The Y parameter of a Quaternion
    ///
    double y() const;
    ///
    /// \brief Return the Z parameter of a Quaternion
    /// \return The Z parameter of a Quaternion
    ///
    double z() const;
    ///
    // \brief Set the k stabilizer
    /// \param newKStab The new value
    ///
    void setKStab(double newKStab);
    ///
    /// \brief Return the k stabilizer 
    /// \return The k stabilizer
    ///
    double kStab() const;

    ///
    /// \brief To use operator "=" on quaternion
    /// \param other The other quaternion
    ///
    template<typename OtherDerived>
    biorbd::utils::Quaternion& operator=(
            const Eigen::MatrixBase <OtherDerived>& other){
        this->Eigen::Vector4d::operator=(other);
        // I don't understand why the next line doesn't SegFault...
        this->m_Kstab = static_cast<biorbd::utils::Quaternion>(other).m_Kstab;
        return *this;
    }
    ///
    /// \brief To use operator "*" on quaternion
    /// \param other The other quaternion
    ///
    biorbd::utils::Quaternion operator*(
            const biorbd::utils::Quaternion& other) const;
    ///
    /// \brief To use operator "*" on quaternion
    /// \param s A double value
    ///
    biorbd::utils::Quaternion operator*(
            double s) const;
    ///
    /// \brief To use operator "*" on quaternion
    /// \param other Float value
    ///
    biorbd::utils::Quaternion operator*(
            float other) const;
    ///
    /// \brief To use operator "+" on quaternion
    /// \param other Other quaternion to add
    ///
    biorbd::utils::Quaternion operator+(
            const biorbd::utils::Quaternion& other) const;
    ///
    /// \brief To use operator "-" on quaternion
    /// \param other Other quaternion to substract
    ///
    biorbd::utils::Quaternion operator-(
            const biorbd::utils::Quaternion& other) const;

    ///
    /// \brief Construct Quaternion
    /// \param angle The angle
    /// \param x The X parameter of the quaternion
    /// \param y The Y parameter of the quaternion
    /// \param z The Z parameter of the quaternion
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromGLRotate (
            double angle,
            double x,
            double y,
            double z,
            double kStab = 1);
    ///
    /// \brief Construct Quaternion from an axis angle
    /// \param angle_rad The angle in radians
    /// \param axis 3d vector of the axis 
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromAxisAngle (
            double angle_rad,
            const biorbd::utils::Vector3d &axis,
            double kStab = 1);
    ///
    /// \brief Construct Quaternion from a RotoTrans matrix
    /// \param mat RotoTrans matrix
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromMatrix (
            const biorbd::utils::RotoTrans &mat,
            double kStab = 1);
    ///
    /// \brief Construct Quaternion from a 3d matrix
    /// \param mat 3d matrix
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromMatrix (
            const Eigen::Matrix3d &mat,
            double kStab = 1);
    ///
    /// \brief Construct Quaternion from ZYX angles
    /// \param zyx_angles A 3D vector of the zyx angles
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromZYXAngles (
            const biorbd::utils::Vector3d &zyx_angles,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from YXZ angles
    /// \param yxz_angles A 3D vector of the YXZ angles
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromYXZAngles (
            const biorbd::utils::Vector3d &yxz_angles,
            double kStab = 1);
    ///
    /// \brief Construct Quaternion from XYZ angles
    /// \param xyz_angles A 3D vector of the YXZ angles
    /// \param kStab The quaternion stabilization (default: 1)
    ///
    static biorbd::utils::Quaternion fromXYZAngles (
            const biorbd::utils::Vector3d &xyz_angles,
            double kStab = 1);

    ///
    /// \brief Quaternion to matrix TODO:
    /// \return Rototrans matrix
    ///
    biorbd::utils::RotoTrans toMatrix() const;

    ///
    /// \brief TODO:
    /// \param alpha TODO:
    /// \param quat Quaternion
    ///
    biorbd::utils::Quaternion slerp (
            double alpha,
            const Quaternion &quat) const;

    /// 
    /// \brief To conjugate
    /// \return Quaternion
    ///
    biorbd::utils::Quaternion conjugate() const;

    ///
    /// \brief TODO:
    /// \param omega 3D vector of the angular velocity
    /// \param dt Time step TODO:
    /// \return Quaternion TODO: 
    ///
    biorbd::utils::Quaternion timeStep (
            const biorbd::utils::Vector3d &omega,
            double dt);

    ///
    /// \brief Return 3D vector TODO:
    /// \param vec 3D vector TODO:
    /// \return 3D vector TODO:
    ///
    biorbd::utils::Vector3d rotate (const biorbd::utils::Vector3d &vec) const;

    ///
    /// \brief Converts a 3d angular velocity vector into a 4d derivative of the components of the quaternion
    /// \param omega the angular velocity
    /// \return a 4d vector containing the derivatives of the 4 components of the quaternion corresponding to omega
    ///
    biorbd::utils::Quaternion omegaToQDot(const biorbd::utils::Vector3d& omega) const;

    ///
    /// \brief Converts a 3d velocity vector expressed in terms of euler angles rate into the 3d angular velocity vector expressed in the fixed parent frame. See # https://davidbrown3.github.io/2017-07-25/EulerAngles/ for correct equations.
    /// \param eulerDot the Euler angle rates
    /// \param euler the Euler angles
    /// \param seq   the Euler angles sequence
    ///
    biorbd::utils::Vector3d  eulerDotToOmega(
            const biorbd::utils::Vector3d &eulerDot, 
            const biorbd::utils::Vector3d &euler,
            const biorbd::utils::String& seq);
    ///
    /// \brief Derivate of a vector
    /// \param w The vector
    ///
    void derivate(const biorbd::utils::Vector &w);
    ///
    /// \brief To normalize
    /// 
    void normalize();

protected:
    double m_Kstab; ///< Stabilization factor for the derivation

};

}}

#endif // BIORBD_UTILS_QUATERNION_H
