#ifndef BIORBD_UTILS_ROTO_TRANS_H
#define BIORBD_UTILS_ROTO_TRANS_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "Utils/Vector3d.h"

namespace RigidBodyDynamics { namespace Math {
struct SpatialTransform;
}}

#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;
class Vector;

///
/// \brief Homogenous matrix to describe translations and rotations simultaneously
///
class BIORBD_API RotoTrans : public Eigen::Matrix4d
{
public:
    ///
    /// \brief Construct RotoTrans matrix
    /// \param matrix 4D identity matrix
    ///
    RotoTrans(
            const Eigen::Matrix4d& matrix = Eigen::Matrix4d::Identity());

    ///
    /// \brief Construct a RotoTrans matrix from another RotoTrans
    /// \param other The other RotoTrans
    ///
    template<typename OtherDerived> RotoTrans(
            const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix4d(other){}

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
    /// \param rot The rotation matrix
    /// \param trans Translation vector
    ///
    RotoTrans(
            const Eigen::Matrix3d& rot,
            const biorbd::utils::Vector3d& trans = biorbd::utils::Vector3d::Zero());

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    RotoTrans(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Get a particular axis of the rotation matrix
    /// \param i The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    biorbd::utils::Vector3d axe(
            int i); // Aller récupérer un axe en particulier

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
    Eigen::Matrix3d rot() const;

    ///
    /// \brief Spatial transform to RotoTrans
    /// \param st The spatial transform
    ///
    biorbd::utils::RotoTrans& SpatialTransform2RotoTrans(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create a RotoTrans from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of RotoTrans
    ///
    biorbd::utils::RotoTrans& combineRotAndTrans(
            const Eigen::Matrix3d& rot,
            const biorbd::utils::Vector3d& trans);

    ///
    /// \brief Create a RotoTrans from Euler angles
    /// \param rot The Euler angles vector
    /// \param trans The translation vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    biorbd::utils::RotoTrans& transformCardanToMatrix(
            const Eigen::VectorXd& rot,
            const biorbd::utils::Vector3d& trans,
            const biorbd::utils::String& seq);

    ///
    /// \brief Get the Euler angles from the rotation matrix
    /// \param rt The RotoTrans matrix
    /// \param seq The rotation sequence
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static biorbd::utils::Vector transformMatrixToCardan(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::String& seq);

    ///
    /// \brief Get the mean of the 4x4 matrices
    /// \param rt The RotoTrans matrices to mean
    /// \return The mean RotoTrans
    ///
    static biorbd::utils::RotoTrans mean(
            const std::vector<biorbd::utils::RotoTrans>&rt);

    ///
    /// \brief Allow the use of operator=
    /// \param other The other rotoTrans matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::RotoTrans& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix4d::operator=(other);
            return *this;
        }

protected:
    ///
    /// \brief Expand 3D vector to 4D (padding with an extra 1)
    /// \param v1 Vector to expand
    ///
    Eigen::Vector4d expand3dTo4d(const biorbd::utils::Vector3d& v1);
};

}}

///
/// \brief To use operator<< to use std::cout
/// \param os osstream
/// \param rt The RotoTrans matrix
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::RotoTrans &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
