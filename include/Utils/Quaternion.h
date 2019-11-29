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

///
/// Description of a quaternion in the format (w, x, y, z)
///
/// The definition for conversions are taken from
/// https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
///
class BIORBD_API Quaternion : public Eigen::Vector4d
{
public:
    ///
    /// \brief Construct Quaternion
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
            double kStabilizer = 1);
    
    ///
    /// \brief Construct Quaternion from another quaternion
    /// \param other Other quaternion
    ///
    Quaternion(
            const biorbd::utils::Quaternion &other);

    ///
    /// \brief Construct Quaternion
    /// \param w The W-Component of quaternion
    /// \param x The X-Component of quaternion
    /// \param y The Y-Component of quaternion
    /// \param z The Z-Component of quaternion
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
            double w,
            double x,
            double y,
            double z,
            double kStabilizer = 1);
    
    ///
    /// \brief Construct Quaternion
    /// \param w The W-Component of quaternion
    /// \param vec3 The vector describing the imaginary part
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
        double w,
        const biorbd::utils::Vector3d &vec3, 
        double kStabilizer = 1);

    ///
    /// \brief Construct Quaternion
    /// \param vec The 4d-vector in the format (w, x, y, z)
    /// \param kStabilizer The value of the kstabilizer
    Quaternion (
        const biorbd::utils::Vector &vec,
        double kStabilizer = 1);

    ///
    /// \brief Return the real part (w) the Quaternion
    /// \return The real part of the Quaternion
    ///
    double w() const;

    ///
    /// \brief Return the X-Component of the imaginary part of the Quaternion
    /// \return The X-Component of the imaginary part of the Quaternion
    ///
    double x() const;

    ///
    /// \brief Return the Y-Component of the imaginary part of the Quaternion
    /// \return The Y-Component of the imaginary part of the Quaternion
    ///
    double y() const;

    ///
    /// \brief Return the Z-Component of the imaginary part of the Quaternion
    /// \return The Z-Component of the imaginary part of the Quaternion
    ///
    double z() const;

    ///
    /// \brief Set the k stabilizer
    /// \param newKStab The new value
    ///
    void setKStab(double newKStab);

    ///
    /// \brief Return the k stabilizer 
    /// \return The k stabilizer
    ///
    /// The k statilizer value is used during several operation to make the
    /// quaternion tending toward a norm of $1$
    ///
    double kStab() const;

    ///
    /// \brief Allows for the operation= assignation
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
    /// \brief Quaternion multiplication
    /// \param other The other quaternion
    ///
    biorbd::utils::Quaternion operator*(
            const biorbd::utils::Quaternion& other) const;

    ///
    /// \brief Multiply the quaternion with a scalar
    /// \param scalar The scalar to multiply with
    ///
    biorbd::utils::Quaternion operator*(
            double scalar) const;

    ///
    /// \brief Multiply the quaternion with a scalar
    /// \param scalar The scalar to multiply with
    ///
    biorbd::utils::Quaternion operator*(
            float scalar) const;

    ///
    /// \brief Add the quaternion to another
    /// \param other The other quaternion to add
    ///
    biorbd::utils::Quaternion operator+(
            const biorbd::utils::Quaternion& other) const;

    ///
    /// \brief Subtract the quaternion to another
    /// \param other Other quaternion to substract
    ///
    biorbd::utils::Quaternion operator-(
            const biorbd::utils::Quaternion& other) const;

    ///
    /// \brief Construct Quaternion from a GL
    /// \param angle The angle in radians
    /// \param x The X-Component of quaternion
    /// \param y The Y-Component of quaternion
    /// \param z The Z-Component of quaternion
    /// \param kStabilizer The value of the kstabilizer
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
    /// \param axis The 3d vector of the axis
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromAxisAngle (
            double angle,
            const biorbd::utils::Vector3d &axis,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from a RotoTrans matrix
    /// \param rt RotoTrans matrix
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromMatrix (
            const biorbd::utils::RotoTrans& rt,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from a Rotation matrix
    /// \param mat The rotation matrix
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromMatrix (
            const Eigen::Matrix3d& mat,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece ZYX)
    /// \param zyx_angles The Euler angles in a sequence where the first element is the Z-component
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromZYXAngles (
            const biorbd::utils::Vector3d &zyx_angles,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece YXZ)
    /// \param zyx_angles The Euler angles in a sequence where the first element is the Y-component
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromYXZAngles (
            const biorbd::utils::Vector3d &yxz_angles,
            double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece XYZ)
    /// \param zyx_angles The Euler angles in a sequence where the first element is the X-component
    /// \param kStabilizer The value of the kstabilizer
    ///
    static biorbd::utils::Quaternion fromXYZAngles (
            const biorbd::utils::Vector3d &xyz_angles,
            double kStab = 1);

    ///
    /// \brief Convert the quaternion to a RotoTrans
    /// \return The rototrans matrix
    ///
    biorbd::utils::RotoTrans toMatrix() const;

    ///
    /// \brief Interpolation of the quaternion between to position
    /// \param alpha The proportion of the rotation
    /// \param quat The quaternion to targe
    ///
    biorbd::utils::Quaternion slerp (
            double alpha,
            const Quaternion &quat) const;

    /// 
    /// \brief Return the conjugate of the quaternion
    /// \return The conjugate of the quaternion
    ///
    biorbd::utils::Quaternion conjugate() const;

    ///
    /// \brief Integrate the quaternion from its velocity
    /// \param omega 3D vector of the angular velocity
    /// \param dt The time step to intergrate on
    /// \return The rotated quaternion
    ///
    biorbd::utils::Quaternion timeStep (
            const biorbd::utils::Vector3d &omega,
            double dt);

    ///
    /// \brief Return a rotated vector from the quaternion
    /// \param vec The vector to rotate
    /// \return The rotated vector
    ///
    biorbd::utils::Vector3d rotate (
            const biorbd::utils::Vector3d &vec) const;

    ///
    /// \brief Converts a 3d angular velocity vector
    /// \param omega the angular velocity
    /// \return a 4d vector containing the derivatives of the 4 components of the quaternion corresponding to omega
    ///
    /// Converts a 3d angular velocity vector into a 4d derivative of
    /// the components of the quaternion
    ///
    biorbd::utils::Quaternion omegaToQDot(const biorbd::utils::Vector3d& omega) const;

    ///
    /// \brief Converts a 3d angular velocity vector
    /// \param eulerDot the Euler angle rates
    /// \param euler the Euler angles
    /// \param seq the Euler angles sequence
    ///
    /// Converts a 3d angular velocity vector expressed in terms of euler
    /// angles rate into the 3d angular velocity vector expressed in the fixed
    /// parent frame. See
    /// https://davidbrown3.github.io/2017-07-25/EulerAngles/
    /// for correct equations.
    ///
    biorbd::utils::Vector3d  eulerDotToOmega(
            const biorbd::utils::Vector3d &eulerDot, 
            const biorbd::utils::Vector3d &euler,
            const biorbd::utils::String& seq);

    ///
    /// \brief Return the time derivative of the quaterion
    /// \param w The vector of time derivative (output)
    ///
    void derivate(
            const biorbd::utils::Vector &w);

    ///
    /// \brief Force the normalization of the quaternion
    /// 
    void normalize();

protected:
    double m_Kstab; ///< Stabilization factor for the derivation

};

}}

#endif // BIORBD_UTILS_QUATERNION_H
