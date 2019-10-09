#ifndef BIORBD_MUSCLES_VIAPOINT_H
#define BIORBD_MUSCLES_VIAPOINT_H

#include "biorbdConfig.h"
#include "Utils/Node3d.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {

class BIORBD_API ViaPoint : public biorbd::utils::Node3d{
public:
    ViaPoint();
    ViaPoint(
            double x,
            double y,
            double z);
    ViaPoint(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name,  // Nom du noeud
            const biorbd::utils::String &parentName);
    ViaPoint(
            const biorbd::utils::Node3d& other);
    ViaPoint(
            const biorbd::muscles::ViaPoint& other);
    biorbd::muscles::ViaPoint DeepCopy() const;
    void DeepCopy(const biorbd::muscles::ViaPoint& other);

    template<typename OtherDerived>
        biorbd::muscles::ViaPoint& operator=(const biorbd::utils::Node3d& other){
            this->biorbd::utils::Node3d::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_MUSCLES_VIAPOINT_H
