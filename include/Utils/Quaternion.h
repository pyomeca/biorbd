#ifndef BIORBD_UTILS_QUATERNION_H
#define BIORBD_UTILS_QUATERNION_H

#include "Utils/Scalar.h"

#include "biorbdConfig.h"
namespace BIORBD_NAMESPACE
{
namespace utils
{
class Vector3d;
class Matrix3d;
class Vector;
class RotoTrans;
class Rotation;
class String;

///
/// Description of a quaternion in the format (w, x, y, z)
///
/// The definition for conversions are taken from
/// https://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
///
#ifdef SWIG
class BIORBD_API Quaternion
#else
class BIORBD_API Quaternion : public RigidBodyDynamics::Math::Vector4d
#endif
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
        const Quaternion &other);

    ///
    /// \brief Construct Quaternion
    /// \param vec4 The vector describing the quaternion
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
        const RigidBodyDynamics::Math::Vector4d &vec4,
        double kStabilizer = 1);

    ///
    /// \brief Construct Quaternion
    /// \param w The W-Component of quaternion
    /// \param x The X-Component of quaternion
    /// \param y The Y-Component of quaternion
    /// \param z The Z-Component of quaternion
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
        const Scalar& w,
        const Scalar& x,
        const Scalar& y,
        const Scalar& z,
        double kStabilizer = 1);

    ///
    /// \brief Construct Quaternion
    /// \param w The W-Component of quaternion
    /// \param vec3 The vector describing the imaginary part
    /// \param kStabilizer The value of the kstabilizer
    ///
    Quaternion (
        const Scalar& w,
        const Vector3d &vec3,
        double kStabilizer = 1);

    ///
    /// \brief Return the real part (w) the Quaternion
    /// \return The real part of the Quaternion
    ///
    Scalar w() const;

    ///
    /// \brief Return the X-Component of the imaginary part of the Quaternion
    /// \return The X-Component of the imaginary part of the Quaternion
    ///
    Scalar x() const;

    ///
    /// \brief Return the Y-Component of the imaginary part of the Quaternion
    /// \return The Y-Component of the imaginary part of the Quaternion
    ///
    Scalar y() const;

    ///
    /// \brief Return the Z-Component of the imaginary part of the Quaternion
    /// \return The Z-Component of the imaginary part of the Quaternion
    ///
    Scalar z() const;

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
    /// quaternion tending toward a norm of \f$1\f$
    ///
    double kStab() const;

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Allows for the operation= assignation
    /// \param other The other quaternion
    ///
    template<typename OtherDerived>
    Quaternion& operator=(
        const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Vector4d::operator=(other);
        // I don't understand why the next line doesn't SegFault...
        this->m_Kstab = static_cast<Quaternion>(other).m_Kstab;
        return *this;
    }

#endif

#endif

    ///
    /// \brief Quaternion multiplication
    /// \param other The other quaternion
    ///
    Quaternion operator*(
        const Quaternion& other) const;

    ///
    /// \brief Multiply the quaternion with a scalar
    /// \param scalar The scalar to multiply with
    ///
    Quaternion operator*(
        const Scalar& scalar) const;

    ///
    /// \brief Multiply the quaternion with a scalar
    /// \param scalar The scalar to multiply with
    ///
    Quaternion operator*(
        float scalar) const;

#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Multiply the quaternion with a scalar
    /// \param scalar The scalar to multiply with
    ///
    Quaternion operator*(
        double scalar) const;
#endif

    ///
    /// \brief Add the quaternion to another
    /// \param other The other quaternion to add
    ///
    Quaternion operator+(
        const Quaternion& other) const;

    ///
    /// \brief Subtract the quaternion to another
    /// \param other Other quaternion to substract
    ///
    Quaternion operator-(
        const Quaternion& other) const;

    ///
    /// \brief Construct Quaternion from a GL
    /// \param angle The angle in radians
    /// \param x The X-Component of quaternion
    /// \param y The Y-Component of quaternion
    /// \param z The Z-Component of quaternion
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromGLRotate (
        const Scalar& angle,
        const Scalar& x,
        const Scalar& y,
        const Scalar& z,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from an axis angle
    /// \param angle The angle in radians
    /// \param axis The 3d vector of the axis
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromAxisAngle (
        const Scalar& angle,
        const Vector3d &axis,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from a RotoTrans matrix
    /// \param rt RotoTrans matrix
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromMatrix (
        const RotoTrans& rt,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from a Rotation matrix
    /// \param mat The rotation matrix
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromMatrix (
        const Rotation &mat,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece ZYX)
    /// \param zyx_angles The Euler angles in a sequence where the first element is the Z-component
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromZYXAngles (
        const Vector3d &zyx_angles,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece YXZ)
    /// \param yxz_angles The Euler angles in a sequence where the first element is the Y-component
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromYXZAngles (
        const Vector3d &yxz_angles,
        double kStab = 1);

    ///
    /// \brief Construct Quaternion from Euler angles (sequece XYZ)
    /// \param xyz_angles The Euler angles in a sequence where the first element is the X-component
    /// \param kStab The value of the kstabilizer
    ///
    static Quaternion fromXYZAngles (
        const Vector3d &xyz_angles,
        double kStab = 1);

    ///
    /// \brief Convert the quaternion to a RotoTrans
    /// \param skipAsserts Check if the norm of the quaternion is approximately 1
    /// \return The rotation matrix
    ///
    /// The function throws a runtime_error if the skipAsserts is false and the
    /// norm of the quaternion is not almost one. In order to accelerate the
    /// computation of the norm, the norm-squared is evaluated. The threshold is
    /// 1e-10 for the norm-squared
    ///
    Rotation toMatrix(
        bool skipAsserts = false) const;

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Interpolation of the quaternion between to position
    /// \param alpha The proportion of the rotation
    /// \param quat The quaternion to targe
    ///
    Quaternion slerp (
        double alpha,
        const Quaternion &quat) const;
#endif

    ///
    /// \brief Return the conjugate of the quaternion
    /// \return The conjugate of the quaternion
    ///
    Quaternion conjugate() const;

    ///
    /// \brief Integrate the quaternion from its velocity
    /// \param omega 3D vector of the angular velocity
    /// \param dt The time step to intergrate on
    /// \return The rotated quaternion
    ///
    Quaternion timeStep (
        const Vector3d &omega,
        double dt);

    ///
    /// \brief Return a rotated vector from the quaternion
    /// \param vec The vector to rotate
    /// \return The rotated vector
    ///
    Vector3d rotate (
        const Vector3d &vec) const;

    ///
    /// \brief Converts a 3d angular velocity vector into a 4d derivative of the components of the quaternion
    /// \param omega the angular velocity
    /// \return a 4d vector containing the derivatives of the 4 components of the quaternion corresponding to omega
    ///
    Quaternion omegaToQuatDot(
        const Vector3d& omega) const;

    ///
    /// \brief Generate the velocity matrix which allows to go from/to euler angles to/from omega(body velocity)
    /// \param euler the Euler angles
    /// \param seq the Euler angles sequence
    /// \return a 3d matrix
    ///
    Matrix3d velocityMatrix (
        const Vector3d &euler,
        const String& seq);
        
    ///
    /// \brief Converts a 3d angular velocity vector
    /// \param eulerDot the Euler angle rates
    /// \param euler the Euler angles
    /// \param seq the Euler angles sequence
    /// \return a 3d vector of the body angular velocity
    ///
    Vector3d  eulerDotToOmega(
        const Vector3d &eulerDot,
        const Vector3d &euler,
        const String& seq);
        
    ///
    /// \brief converts a 3d vector of the body angular velocity (omega) into a 3d vector of the euler angles rate.
    /// \param euler the Euler angles
    /// \param w the body velocity (omega)
    /// \param seq the Euler angles sequence
    /// \return a 3d vector of the euler angles rate
    ///
    Vector3d omegaToEulerDot(
    const Vector3d &euler,
    const Vector3d &w,
    const String& seq);

    ///
    /// \brief Return the time derivative of the quaterion
    /// \param w The vector of time derivative (output)
    ///
    void derivate(
        const Vector &w);

    ///
    /// \brief Force the normalization of the quaternion
    ///
    void normalize();

protected:
    double m_Kstab; ///< Stabilization factor for the derivation

};

}
}

#endif // BIORBD_UTILS_QUATERNION_H
