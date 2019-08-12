#ifndef S2M_MUSCLE_PATH_CHANGER_H
#define S2M_MUSCLE_PATH_CHANGER_H

#include "biorbdConfig.h"
#include "s2mNodeMuscle.h"
#include "Utils/String.h"

class BIORBD_API s2mMusclePathChanger : public s2mNodeMuscle
{
public:
    s2mMusclePathChanger(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~s2mMusclePathChanger() = 0;
    const biorbd::utils::String& type() const;

protected:
    biorbd::utils::String m_type;
};

#endif // S2M_MUSCLE_PATH_CHANGER_H
