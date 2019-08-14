#ifndef S2M_VIAPOINT_H
#define S2M_VIAPOINT_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Muscles/PathChanger.h"

class BIORBD_API s2mViaPoint : public s2mMusclePathChanger{
public:
    s2mViaPoint(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~s2mViaPoint();

};

#endif // S2M_VIAPOINT_H
