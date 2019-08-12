#ifndef S2M_ATTITUDE_H
#define S2M_ATTITUDE_H

#include <vector>
#include <Eigen/Dense>
#include <rbdl/rbdl_math.h>
#include "biorbdConfig.h"

class s2mNode;
class s2mString;
namespace biorbd { namespace utils {
class BIORBD_API Attitude : public Eigen::Matrix4d
{
public:
    Attitude(const Eigen::Matrix4d& = Eigen::Matrix4d::Identity());
    Attitude(
            const Eigen::VectorXd&,
            const Eigen::Vector3d&,
            const s2mString&);
    Attitude(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans = Eigen::Vector3d::Zero());
    Attitude(const RigidBodyDynamics::Math::SpatialTransform&);

    Eigen::Vector3d axe(int); // Aller récupérer un axe en particulier

    Attitude transpose() const;
    Eigen::Vector3d trans() const;
    Eigen::Matrix3d rot() const;
    void setIdentity();
    bool isIdentity();

    static Attitude SpatialTransform2Attitude(const RigidBodyDynamics::Math::SpatialTransform& st);
    static Attitude combineRotAndTrans(
            const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& trans);
    static Eigen::Matrix4d transformCardanToMatrix(
            const Eigen::VectorXd&,
            const Eigen::Vector3d&,
            const s2mString&);
    static Eigen::VectorXd transformMatrixToCardan(
            const Attitude&,
            const s2mString &seq);

    const Attitude operator*(const Attitude&);
    const Eigen::Vector3d operator*(const Eigen::Vector3d&);
    const Eigen::Vector3d operator*(const Eigen::Vector4d&);
    const s2mNode operator*(const s2mNode&);

    static Attitude mean(const std::vector<Attitude>&); // Moyenne des matrices 4x4
protected:
    Eigen::Vector4d expand3dTo4d(const Eigen::Vector3d&);

};
std::ostream& operator<<(std::ostream& os, const Attitude &a);

}}

#endif // S2M_ATTITUDE_H
