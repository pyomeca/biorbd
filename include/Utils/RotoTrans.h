#ifndef BIORBD_UTILS_ROTO_TRANS_H
#define BIORBD_UTILS_ROTO_TRANS_H

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
class Rotation;
class Vector3d;

///
/// \brief Homogenous matrix to describe translations and rotations simultaneously
///
class BIORBD_API RotoTrans : public RigidBodyDynamics::Math::Matrix4d
{
public:
    ///
    /// \brief Construct RotoTrans matrix
    /// \param matrix 4D identity matrix
    ///
    RotoTrans(
            const RigidBodyDynamics::Math::Matrix4d& matrix = RigidBodyDynamics::Math::Matrix4d::Identity());

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a RotoTrans matrix from another RotoTrans
    /// \param other The other RotoTrans
    ///
    template<typename OtherDerived> RotoTrans(
            const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix4d(other){
        checkUnitary();
    }
#endif

    ///
    /// \brief Contruct Rototrans
    /// \param rot The rotation matrix
    ///
    RotoTrans(
            const biorbd::utils::Rotation& rot);

    ///
    /// \brief Contruct Rototrans
    /// \param rot The rotation matrix
    /// \param trans Translation vector
    ///
    RotoTrans(
            const biorbd::utils::Rotation& rot,
            const biorbd::utils::Vector3d& trans);

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
            const biorbd::utils::Vector& rotation,
            const biorbd::utils::Vector3d& translation,
            const biorbd::utils::String &rotationSequence);

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    RotoTrans(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes and an origin defined by markers
    /// \param origin The position of the origin
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static biorbd::utils::RotoTrans fromMarkers(
            const biorbd::rigidbody::NodeSegment& origin,
            const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>& axis1markers,
            const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>& axis2markers,
            const std::pair<biorbd::utils::String, biorbd::utils::String> &axesNames,
            const biorbd::utils::String& axisToRecalculate);

    ///
    /// \brief Get a particular axis of the rotation matrix
    /// \param idx The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    biorbd::utils::Vector3d axe(
            unsigned int idx) const ;

    ///
    /// \brief Return the tranposed matrix
    /// \return The transposed matrix
    ///
    biorbd::utils::RotoTrans transpose() const;

    ///
    /// \brief Return the translation vector
    /// \return The translation vector
    ///
    biorbd::utils::Vector3d trans() const;

    ///
    /// \brief Return the rotation matrix
    /// \return The rotation matrix
    ///
    biorbd::utils::Rotation rot() const;

    ///
    /// \brief Set the RotoTrans from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of RotoTrans
    ///
    biorbd::utils::RotoTrans& combineRotAndTrans(
            const biorbd::utils::Rotation& rot,
            const biorbd::utils::Vector3d& trans);

    ///
    /// \brief set the RotoTrans from a spatial transform
    /// \param st The spatial transform
    /// \return The matrix of RotoTrans
    ///
    biorbd::utils::RotoTrans& fromSpatialTransform(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create a RotoTrans from Euler angles
    /// \param rot The Euler angles vector
    /// \param trans The translation vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    biorbd::utils::RotoTrans& fromEulerAngles(
            const biorbd::utils::Vector &rot,
            const biorbd::utils::Vector3d& trans,
            const biorbd::utils::String& seq);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param rt The RotoTrans matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static biorbd::utils::Vector toEulerAngles(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::String &seq);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Get the mean of the 4x4 matrices
    /// \param rt The RotoTrans matrices to mean
    /// \return The mean RotoTrans
    ///
    static biorbd::utils::RotoTrans mean(
            const std::vector<biorbd::utils::RotoTrans>&rt);
#endif

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use of operator=
    /// \param other The other rotoTrans matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::RotoTrans& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix4d::operator=(other);
            return *this;
        }
#endif

protected:
    ///
    /// \brief Expand 3D vector to 4D (padding with an extra 1)
    /// \param v1 Vector to expand
    ///
    RigidBodyDynamics::Math::Vector4d expand3dTo4d(const biorbd::utils::Vector3d& v1);

    ///
    /// \brief Check if the RotoTrans has a unitary matrix of rotation and the last row is (0, 0, 0, 1)
    ///
    /// That function throws a runtime_error if the check fails
    ///
    void checkUnitary();
};

}}

///
/// \brief To use operator<< to use std::cout
/// \param os osstream
/// \param rt The RotoTrans matrix
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::RotoTrans &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
