#ifndef BIORBD_UTILS_ROTATION_H
#define BIORBD_UTILS_ROTATION_H

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace RigidBodyDynamics { namespace Math {
struct SpatialTransform;
}}

#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Vector;

///
/// \brief Rotation matrix
///
class BIORBD_API Rotation : public Eigen::Matrix3d
{
public:
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
            const Eigen::Matrix3d& = Eigen::Matrix3d::Identity());

    ///
    /// \brief Construct a Rotation matrix from another Rotation
    /// \param other The other Rotation
    ///
    template<typename OtherDerived> Rotation(
            const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix3d(other){
        checkUnitary();
    }

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
    biorbd::utils::Vector axe(
            int idx);

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
            const Eigen::VectorXd& rot,
            const biorbd::utils::String& seq);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param rt The Rotation matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static  biorbd::utils::Vector toEulerAngles(
            const biorbd::utils::Rotation& rt,
            const biorbd::utils::String& seq);

    ///
    /// \brief Get the mean of the Rotation matrices
    /// \param mToMean The Rotation matrices to mean
    /// \return The mean Rotation matrix
    ///
    static biorbd::utils::Rotation mean(
            const std::vector<biorbd::utils::Rotation>& mToMean);

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
