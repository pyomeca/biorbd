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

    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &m,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool  = true) = 0;
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans&,
            const biorbd::utils::Node3d&,
            const biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            double* muscleLength = nullptr) = 0 ; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::utils::Node3d&,
            const biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            double* muscleLength = nullptr) = 0; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            double* muscleLength = nullptr) = 0; // Assume un appel déja faits

    biorbd::muscles::WrappingObject& operator=(const biorbd::utils::Node3d& other){
        this->biorbd::utils::Node3d::operator=(other);
        return *this;
    }
protected:
    std::shared_ptr<biorbd::utils::RotoTrans> m_RT;
};

}}

#endif // BIORBD_MUSCLES_WRAPPING_OBJECT_H
