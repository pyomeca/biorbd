#ifndef BIORBD_UTILS_ROTO_TRANS_H
#define BIORBD_UTILS_ROTO_TRANS_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "Utils/Node3d.h"

namespace RigidBodyDynamics { namespace Math {
struct SpatialTransform;
}}

#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class String;

///
/// \brief The RotoTrans class is the 3d position given in a 4d matrix
///
class BIORBD_API RotoTrans : public Eigen::Matrix4d
{
public:
    RotoTrans(const Eigen::Matrix4d& = Eigen::Matrix4d::Identity());
    template<typename OtherDerived> RotoTrans(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix4d(other){}
    RotoTrans(
            const Eigen::VectorXd& rotation,
            const Eigen::Vector3d& translation,
            const biorbd::utils::String &rotationSequence);
    RotoTrans(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans = Eigen::Vector3d::Zero());
    RotoTrans(const RigidBodyDynamics::Math::SpatialTransform&);

    Eigen::Vector3d axe(int); // Aller récupérer un axe en particulier

    biorbd::utils::RotoTrans transpose() const;
    Eigen::Vector3d trans() const;
    Eigen::Matrix3d rot() const;

    biorbd::utils::RotoTrans& SpatialTransform2RotoTrans(const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create an RotoTrans matrix from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of RotoTrans
    ///
    biorbd::utils::RotoTrans& combineRotAndTrans(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans);

    biorbd::utils::RotoTrans& transformCardanToMatrix(
            const Eigen::VectorXd&,
            const Eigen::Vector3d&,
            const biorbd::utils::String&);
    static Eigen::VectorXd transformMatrixToCardan(
            const biorbd::utils::RotoTrans&,
            const biorbd::utils::String &seq);

    static biorbd::utils::RotoTrans mean(const std::vector<biorbd::utils::RotoTrans>&); // Moyenne des matrices 4x4

    template<typename OtherDerived>
        biorbd::utils::RotoTrans& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix4d::operator=(other);
            return *this;
        }
protected:
    Eigen::Vector4d expand3dTo4d(const Eigen::Vector3d&);
};

}}

std::ostream& operator<<(std::ostream& os, const biorbd::utils::RotoTrans &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
