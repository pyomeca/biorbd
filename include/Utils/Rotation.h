#ifndef BIORBD_UTILS_ROTATION_H
#define BIORBD_UTILS_ROTATION_H

#include <vector>
#include <memory>
#include "rbdl_math.h"

namespace RigidBodyDynamics { namespace Math {
struct SpatialTransform;
}}

#include "biorbdConfig.h"

namespace biorbd {
namespace rigidbody {
class NodeSegment;
}

namespace utils {
class String;
class Vector;
class Vector3d;

///
/// \brief Rotation matrix
///
class BIORBD_API Rotation : public RigidBodyDynamics::Math::Matrix3d
{
public:
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
            const RigidBodyDynamics::Math::Matrix3d& matrix = RigidBodyDynamics::Math::Matrix3d::Identity());

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a Rotation matrix from another Rotation
    /// \param other The other Rotation
    ///
    template<typename OtherDerived> Rotation(
            const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix3d(other){
        checkUnitary();
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
            const RigidBodyDynamics::Math::MatrixNd& m);
#endif

    Rotation(RigidBodyDynamics::Math::Scalar v00, RigidBodyDynamics::Math::Scalar v01, RigidBodyDynamics::Math::Scalar v02,
             RigidBodyDynamics::Math::Scalar v10, RigidBodyDynamics::Math::Scalar v11, RigidBodyDynamics::Math::Scalar v12,
             RigidBodyDynamics::Math::Scalar v20, RigidBodyDynamics::Math::Scalar v21, RigidBodyDynamics::Math::Scalar v22);

    ///
    /// \brief Contruct a Rotation matrix
    /// \param rotation The Euler angles vector
    /// \param rotationSequence The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    Rotation(
            const biorbd::utils::Vector& rotation,
            const biorbd::utils::String& rotationSequence);

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    Rotation(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Get a particular axis of the Rotation matrix
    /// \param idx The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    biorbd::utils::Vector3d axe(
            unsigned int idx) const;

    ///
    /// \brief set the Rotation from a spatial transform
    /// \param st The spatial transform
    /// \return The matrix of Rotation
    ///
    static biorbd::utils::Rotation fromSpatialTransform(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create a Rotation from Euler angles
    /// \param rot The Euler angles vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    biorbd::utils::Rotation& fromEulerAngles(
            const biorbd::utils::Vector& rot,
            const biorbd::utils::String& seq);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes defined by markers
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static biorbd::utils::Rotation fromMarkers(
            const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>& axis1markers,
            const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>& axis2markers,
            const std::pair<biorbd::utils::String, biorbd::utils::String> &axesNames,
            const biorbd::utils::String& axisToRecalculate);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param r The Rotation matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static  biorbd::utils::Vector toEulerAngles(
            const biorbd::utils::Rotation& r,
            const biorbd::utils::String& seq);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Get the mean of the Rotation matrices
    /// \param mToMean The Rotation matrices to mean
    /// \return The mean Rotation matrix
    ///
    static biorbd::utils::Rotation mean(
            const std::vector<biorbd::utils::Rotation>& mToMean);
#endif

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use of operator=
    /// \param other The other Rotation matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::Rotation& operator=(
                const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix3d::operator=(other);
            return *this;
        }
#endif

protected:
    ///
    /// \brief Check if the Rotation is a unitary matrix of rotation
    ///
    /// That function throws a runtime_error if the check fails
    ///
    void checkUnitary();
};

}}

///
/// \brief To use operator<< to use std::cout
/// \param os osstream
/// \param rt The Rotation matrix
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::Rotation &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
