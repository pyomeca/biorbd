#ifndef BIORBD_MUSCLES_PATH_CHANGER_H
#define BIORBD_MUSCLES_PATH_CHANGER_H

#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Muscles/MuscleNode.h"

namespace biorbd {
namespace muscles {

class BIORBD_API PathChanger : public biorbd::muscles::MuscleNode
{
public:
    PathChanger(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~PathChanger() = 0;
    const biorbd::utils::String& type() const;

protected:
    biorbd::utils::String m_type;

};

}}

#endif // BIORBD_MUSCLES_PATH_CHANGER_H
