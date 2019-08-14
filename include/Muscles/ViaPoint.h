#ifndef BIORBD_MUSCLES_VIAPOINT_H
#define BIORBD_MUSCLES_VIAPOINT_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Muscles/PathChanger.h"

namespace biorbd { namespace muscles {

class BIORBD_API ViaPoint : public biorbd::muscles::PathChanger{
public:
    ViaPoint(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~ViaPoint();

};

}}

#endif // BIORBD_MUSCLES_VIAPOINT_H
