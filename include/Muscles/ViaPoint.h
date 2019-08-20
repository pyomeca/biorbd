#ifndef BIORBD_MUSCLES_VIAPOINT_H
#define BIORBD_MUSCLES_VIAPOINT_H

#include "biorbdConfig.h"
#include "Muscles/PathChanger.h"

namespace biorbd {
namespace muscles {

class BIORBD_API ViaPoint : public biorbd::muscles::PathChanger{
public:
    ViaPoint(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    ViaPoint(
            const biorbd::muscles::MuscleNode &v, // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~ViaPoint();

};

}}

#endif // BIORBD_MUSCLES_VIAPOINT_H
