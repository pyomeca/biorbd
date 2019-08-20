#ifndef BIORBD_UTILS_ATTITUDE_H
#define BIORBD_UTILS_ATTITUDE_H

#include <vector>
#include <Eigen/Dense>

namespace RigidBodyDynamics { namespace Math {
struct SpatialTransform;
}}

#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
class String;

///
/// \brief The Attitude class is the 3d position given in a 4d matrix
///
class BIORBD_API Attitude : public Eigen::Matrix4d
{
public:
    Attitude(const Eigen::Matrix4d& = Eigen::Matrix4d::Identity());
    Attitude(
            const Eigen::VectorXd&,
            const Eigen::Vector3d&,
            const biorbd::utils::String&);
    Attitude(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans = Eigen::Vector3d::Zero());
    Attitude(const RigidBodyDynamics::Math::SpatialTransform&);

    Eigen::Vector3d axe(int); // Aller récupérer un axe en particulier

    biorbd::utils::Attitude transpose() const;
    Eigen::Vector3d trans() const;
    Eigen::Matrix3d rot() const;
    void setIdentity();
    bool isIdentity();

    static biorbd::utils::Attitude SpatialTransform2Attitude(const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create an Attitude matrix from a rotation and a translation
    /// \param rot The matrix of rotation
    /// \param trans The vector of translation
    /// \return The matrix of attitude
    ///
    static biorbd::utils::Attitude combineRotAndTrans(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans);

    static Eigen::Matrix4d transformCardanToMatrix(
            const Eigen::VectorXd&,
            const Eigen::Vector3d&,
            const biorbd::utils::String&);
    static Eigen::VectorXd transformMatrixToCardan(
            const biorbd::utils::Attitude&,
            const biorbd::utils::String &seq);

    const biorbd::utils::Attitude operator*(const biorbd::utils::Attitude&);
    const Eigen::Vector3d operator*(const Eigen::Vector3d&);
    const Eigen::Vector3d operator*(const Eigen::Vector4d&);
    const biorbd::utils::Node3d operator*(const biorbd::utils::Node3d&);

    static biorbd::utils::Attitude mean(const std::vector<biorbd::utils::Attitude>&); // Moyenne des matrices 4x4

protected:
    Eigen::Vector4d expand3dTo4d(const Eigen::Vector3d&);

};

}}

std::ostream& operator<<(std::ostream& os, const biorbd::utils::Attitude &a);

#endif // BIORBD_UTILS_ATTITUDE_H
