#ifndef BIORBD_UTILS_ROTATION_H
#define BIORBD_UTILS_ROTATION_H

#include <vector>
#include <memory>
#include "Utils/Matrix3d.h"

#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace rigidbody
{
class NodeSegment;
}

namespace utils
{
class SpatialTransform;
class String;
class Vector;
class Vector3d;

///
/// \brief Rotation matrix
///
#ifdef SWIG
class BIORBD_API Rotation
#else
class BIORBD_API Rotation : public Matrix3d
#endif
{
public:
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
        const Matrix3d& matrix = Matrix3d::Identity());

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a Rotation matrix from another Rotation
    /// \param other The other Rotation
    ///
    template<typename OtherDerived> Rotation(
        const Eigen::MatrixBase<OtherDerived>& other) :
        Matrix3d(other)
    {
        checkUnitary();
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    Rotation(
        const RBDLCasadiMath::MX_Xd_static<3, 3>& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    Rotation(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif

    ///
    /// \brief Rotation Construct a Rotation matrix by elements
    /// \param v00 Row 0, Col 0
    /// \param v01 Row 0, Col 1
    /// \param v02 Row 0, Col 2
    /// \param v10 Row 1, Col 0
    /// \param v11 Row 1, Col 1
    /// \param v12 Row 1, Col 2
    /// \param v20 Row 2, Col 0
    /// \param v21 Row 2, Col 1
    /// \param v22 Row 2, Col 2
    ///
    Rotation(const Scalar& v00, const Scalar& v01,
             const Scalar& v02,
             const Scalar& v10, const Scalar& v11,
             const Scalar& v12,
             const Scalar& v20, const Scalar& v21,
             const Scalar& v22);

    ///
    /// \brief Contruct a Rotation matrix
    /// \param rotation The Euler angles vector
    /// \param rotationSequence The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    Rotation(
        const Vector& rotation,
        const String& rotationSequence);

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    Rotation(
        const utils::SpatialTransform& st);

    ///
    /// \brief Get a particular axis of the Rotation matrix
    /// \param idx The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    Vector3d axe(
        size_t idx) const;

    ///
    /// \brief set the Rotation from a spatial transform
    /// \param st The spatial transform
    /// \return The matrix of Rotation
    ///
    static Rotation fromSpatialTransform(
        const utils::SpatialTransform& st);

    ///
    /// \brief Create a Rotation from Euler angles
    /// \param rot The Euler angles vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    static Rotation fromEulerAngles(
        const Vector& rot,
        const String& seq);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes defined by markers
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static Rotation fromMarkersNonNormalized(
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>& axis1markers,
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>& axis2markers,
        const std::pair<String, String> &axesNames,
        const String& axisToRecalculate);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes defined by markers
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static Rotation fromMarkers(
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>&
        axis1markers,
        const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment>&
        axis2markers,
        const std::pair<String, String> &axesNames,
        const String& axisToRecalculate);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param r The Rotation matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static  Vector toEulerAngles(
        const Rotation& r,
        const String& seq);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Get the mean of the Rotation matrices
    /// \param mToMean The Rotation matrices to mean
    /// \return The mean Rotation matrix
    ///
    static utils::Rotation mean(
        const std::vector<utils::Rotation>& mToMean);
#endif

#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use of operator=
    /// \param other The other Rotation matrix
    ///
    template<typename OtherDerived>
    Rotation& operator=(
        const Eigen::MatrixBase <OtherDerived>& other)
    {
        Eigen::Matrix3d::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_static<3, 3>& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif
#endif

protected:
    ///
    /// \brief Check if the Rotation is a unitary matrix of rotation
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
    /// \param rt The Rotation matrix
    ///
    std::ostream& operator<<(std::ostream& os, const BIORBD_NAMESPACE::utils::Rotation &rt);
#endif

#endif // BIORBD_UTILS_ROTO_TRANS_H
