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
/// \brief The RotoTrans class is the 3d position given in a 4d matrix
///
class BIORBD_API RotoTrans : public Eigen::Matrix4d
{
public:
    ///
    /// \brief Construct RotoTrans matrix
    /// \param m 4D identity matrix
    ///
    RotoTrans(const Eigen::Matrix4d&m = Eigen::Matrix4d::Identity());

    ///
    /// \brief Construct RotoTrans matrix
    /// \param other TODO
    ///
    template<typename OtherDerived> RotoTrans(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix4d(other){}

    ///
    /// \brief Contruct Rototrans
    /// \param rotation Rotation vector
    /// \param translation Translation vector
    /// \param rotationSequence The rotation sequence
    ///
    RotoTrans(
            const biorbd::utils::Vector& rotation,
            const biorbd::utils::Vector3d& translation,
            const biorbd::utils::String &rotationSequence);
    ///
    /// \brief Contruct Rototrans
    /// \param rot Rotation 3D matrix
    /// \param trans Translation vector
    ///
    RotoTrans(
            const Eigen::Matrix3d& rot,
            const biorbd::utils::Vector3d& trans = biorbd::utils::Vector3d::Zero());

    RotoTrans(
            const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Get a particular axis
    /// \param i Number of axis (must be between 0 and 2)
    /// \return The data on the axis
    ///
    biorbd::utils::Vector3d axe(int i); // Aller récupérer un axe en particulier

    ///
    /// \brief Tranpose matrix
    /// \return The transpose matrix
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
    /// \brief Create an RotoTrans matrix from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of RotoTrans
    ///
    biorbd::utils::RotoTrans& combineRotAndTrans(
            const Eigen::Matrix3d& rot,
            const biorbd::utils::Vector3d& trans);

    ///
    /// \brief Transform Cardan to Matrix
    /// \param rot The rotation vector
    /// \param trans The translation vector
    /// \param seq The rotation sequence
    ///
    biorbd::utils::RotoTrans& transformCardanToMatrix(
            const Eigen::VectorXd& rot,
            const biorbd::utils::Vector3d& trans,
            const biorbd::utils::String& seq);


    ///
    /// \brief Transform matrix to cardan
    /// \param rt The RotoTrans matrix
    /// \param seq The rotation sequence
    ///
    static biorbd::utils::Vector transformMatrixToCardan(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::String& seq);

    ///
    /// \brief Get the mean of the 4x4 matrices
    /// \param rt The rototrans matrix
    ///
    static biorbd::utils::RotoTrans mean(const std::vector<biorbd::utils::RotoTrans>&rt); // Moyenne des matrices 4x4
    ///
    /// \brief To use operator "=" 
    /// \param other The other rotoTrans matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::RotoTrans& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix4d::operator=(other);
            return *this;
        }
protected:
    ///
    /// \brief Expand 3D vector to 4D
    /// \param v1 Vector to expand
    ///
    Eigen::Vector4d expand3dTo4d(const biorbd::utils::Vector3d&v1);
};

}}
///
/// \brief To use operator "<<" 
/// \param os O stream
/// \param rt The rototrans matrix
///
std::ostream& operator<<(std::ostream& os, const biorbd::utils::RotoTrans &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
