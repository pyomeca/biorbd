#ifndef BIORBD_MUSCLES_WRAPPING_OBJECT_H
#define BIORBD_MUSCLES_WRAPPING_OBJECT_H

#include "biorbdConfig.h"
#include "Utils/Node3d.h"

namespace biorbd {
namespace utils {
class String;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {

class BIORBD_API WrappingObject : public biorbd::utils::Node3d
{
public:
    WrappingObject();
    WrappingObject(
            double x,
            double y,
            double z);
    WrappingObject(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name,  // Nom du noeud
            const biorbd::utils::String &parentName);
    WrappingObject(
            const biorbd::utils::Node3d& other);
    WrappingObject(
            const Eigen::Vector3d& other,
            const biorbd::utils::String& name,
            const biorbd::utils::String& parentName);

    virtual void wrapPoints(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* muscleLength = nullptr) = 0 ; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* muscleLength = nullptr) = 0; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* muscleLength = nullptr) = 0; // Assume un appel déja faits

    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool updateKin = true) = 0;
    const biorbd::utils::RotoTrans& RT() const;

    biorbd::muscles::WrappingObject& operator=(const biorbd::utils::Node3d& other){
        this->biorbd::utils::Node3d::operator=(other);
        return *this;
    }
protected:
    std::shared_ptr<biorbd::utils::RotoTrans> m_RT;
};

}}

#endif // BIORBD_MUSCLES_WRAPPING_OBJECT_H
