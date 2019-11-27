#ifndef BIORBD_UTILS_ROTATION_H
#define BIORBD_UTILS_ROTATION_H

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
class Vector;


class BIORBD_API Rotation : public Eigen::Matrix3d
{
public:
    Rotation(const Eigen::Matrix3d& = Eigen::Matrix3d::Identity());
    template<typename OtherDerived> Rotation(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Matrix3d(other){}
    Rotation(
            const biorbd::utils::Vector& rotation,
            //const biorbd::utils::Node3d& translation,
            const biorbd::utils::String &rotationSequence);
    Rotation(
        const Eigen::Matrix3d& rot);
            ///const biorbd::utils::Node3d& trans = Eigen::Vector3d::Zero());
    Rotation(const RigidBodyDynamics::Math::SpatialTransform&);

    biorbd::utils::Vector axe(int); // Aller récupérer un axe en particulier

    biorbd::utils::Rotation transpose() const;
    Eigen::Matrix3d rot() const;

   // biorbd::utils::Rotation& SpatialTransform2RotoTrans(const RigidBodyDynamics::Math::SpatialTransform& st);


    biorbd::utils::RotoTrans& transformCardanToMatrix(
            const Eigen::VectorXd& rot,
           /// const Eigen::Vector3d& trans,
            const biorbd::utils::String& seq);
    static  biorbd::utils::Vector transformMatrixToCardan(
            const biorbd::utils::Rotation& rt,
            const biorbd::utils::String &seq);

    static biorbd::utils::Rotation mean(const std::vector<biorbd::utils::Rotation>&); // Moyenne des matrices 3x3

    template<typename OtherDerived>
        biorbd::utils::Rotation& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            Eigen::Matrix3d::operator=(other);
            return *this;
        }
protected:
    //Do we need it?
   // Eigen::Vector4d expand3dTo4d(const Eigen::Vector3d&);
};

}}

std::ostream& operator<<(std::ostream& os, const biorbd::utils::Rotation &rt);

#endif // BIORBD_UTILS_ROTO_TRANS_H
