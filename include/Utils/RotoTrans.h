#ifndef BIORBD_UTILS_ROTO_TRANS_H
#define BIORBD_UTILS_ROTO_TRANS_H

#include <vector>
#include <memory>
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace rigidbody
{
class NodeSegment;
}

namespace utils
{
class String;
class Vector;
class Rotation;
class Vector3d;
class SpatialTransform;

///
/// \brief Homogenous matrix to describe translations and rotations simultaneously
///
#ifdef SWIG
class BIORBD_API RotoTrans
#else
class BIORBD_API RotoTrans : public RigidBodyDynamics::Math::Matrix4d
#endif
{
public:
    ///
    /// \brief Construct RotoTrans matrix
    /// \param matrix 4D identity matrix
    ///
    RotoTrans(
        const RigidBodyDynamics::Math::Matrix4d& matrix =
            RigidBodyDynamics::Math::Matrix4d::Identity());

    ///
    /// \brief RotoTrans Construct a RotoTrans matrix by elements
    /// \param v00 Row 0, Col 0
    /// \param v01 Row 0, Col 1
    /// \param v02 Row 0, Col 2
    /// \param v03 Row 0, Col 3
    /// \param v10 Row 1, Col 0
    /// \param v11 Row 1, Col 1
    /// \param v12 Row 1, Col 2
    /// \param v13 Row 1, Col 3
    /// \param v20 Row 2, Col 0
    /// \param v21 Row 2, Col 1
    /// \param v22 Row 2, Col 2
    /// \param v23 Row 2, Col 3
    /// \param v30 Row 3, Col 0
    /// \param v31 Row 3, Col 1
    /// \param v32 Row 3, Col 2
    /// \param v33 Row 3, Col 3
    ///
    RotoTrans(
        const Scalar& v00, const Scalar& v01, const Scalar& v02, const Scalar& v03,
        const Scalar& v10, const Scalar& v11, const Scalar& v12, const Scalar& v13,
        const Scalar& v20, const Scalar& v21, const Scalar& v22, const Scalar& v23,
        const Scalar& v30, const Scalar& v31, const Scalar& v32, const Scalar& v33
    );

    ///
    /// \brief Contruct Rototrans
    /// \param rot The rotation matrix
    ///
    RotoTrans(
        const Rotation& rot);

    ///
    /// \brief Contruct Rototrans
    /// \param rot The rotation matrix
    /// \param trans Translation vector
    ///
    RotoTrans(
        const Rotation& rot,
        const Vector3d& trans);

    ///
    /// \brief Contruct Rototrans
    /// \param rotation The Euler angles vector
    /// \param translation The translation vector
    /// \param rotationSequence The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    RotoTrans(
        const Vector& rotation,
        const Vector3d& translation,
        const String &rotationSequence);

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    RotoTrans(
        const utils::SpatialTransform& st);

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Construct a RotoTrans matrix from another RotoTrans
    /// \param other The other RotoTrans
    ///
    template<typename OtherDerived> RotoTrans(
        const Eigen::MatrixBase<OtherDerived>& other) :
        RigidBodyDynamics::Math::Matrix4d(other)
    {
        checkUnitary();
    }

#endif

    ///
    /// \brief fromMarkers Creates a system of axes from two axes and an origin defined by markers
    /// \param origin The position of the origin
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static RotoTrans fromMarkers(
        const rigidbody::NodeSegment& origin,
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>&axis1markers,
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>&axis2markers,
        const std::pair<String, String> &axesNames,
        const String& axisToRecalculate);

    ///
    /// \brief Get a particular axis of the rotation matrix
    /// \param idx The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    Vector3d axe(
        size_t idx) const ;

    ///
    /// \brief Return the tranposed matrix
    /// \return The transposed matrix
    ///
    RotoTrans transpose() const;

    ///
    /// \brief Return the translation vector
    /// \return The translation vector
    ///
    Vector3d trans() const;

    ///
    /// \brief Return the rotation matrix
    /// \return The rotation matrix
    ///
    Rotation rot() const;

    ///
    /// \brief Set the RotoTrans from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of RotoTrans
    ///
    static RotoTrans combineRotAndTrans(
        const Rotation& rot,
        const Vector3d& trans);

    ///
    /// \brief set the RotoTrans from a spatial transform
    /// \param st The spatial transform
    /// \return The matrix of RotoTrans
    ///
    static RotoTrans fromSpatialTransform(
        const utils::SpatialTransform& st);

    ///
    /// \brief Create a RotoTrans from Euler angles
    /// \param rot The Euler angles vector
    /// \param trans The translation vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation umust match the number of axes in the rotation
    /// sequence
    ///
    static RotoTrans fromEulerAngles(
        const Vector &rot,
        const Vector3d& trans,
        const String& seq);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param rt The RotoTrans matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static Vector toEulerAngles(
        const RotoTrans& rt,
        const String &seq);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Get the mean of the 4x4 matrices
    /// \param rt The RotoTrans matrices to mean
    /// \return The mean RotoTrans
    ///
    static utils::RotoTrans mean(
        const std::vector<utils::RotoTrans>&rt);
#endif

#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use of operator=
    /// \param other The other rotoTrans matrix
    ///
    template<typename OtherDerived>
    RotoTrans& operator=(const Eigen::MatrixBase <OtherDerived>&
                                        other)
    {
        Eigen::Matrix4d::operator=(other);
        return *this;
    }
#endif
#endif

protected:
    ///
    /// \brief Expand 3D vector to 4D (padding with an extra 1)
    /// \param v1 Vector to expand
    ///
    RigidBodyDynamics::Math::Vector4d expand3dTo4d(const Vector3d&
            v1);

public:
    ///
    /// \brief Check if the RotoTrans has a unitary matrix of rotation and the last row is (0, 0, 0, 1)
    ///
    /// That function throws a runtime_error if the check fails
    ///
    void checkUnitary();
};

}
}

#ifndef SWIG
    ///
    /// \brief To use operator<< to use std::cout
    /// \param os osstream
    /// \param rt The RotoTrans matrix
    ///
    std::ostream& operator<<(std::ostream& os, const BIORBD_NAMESPACE::utils::RotoTrans &rt);
#endif

#endif // BIORBD_UTILS_ROTO_TRANS_H
