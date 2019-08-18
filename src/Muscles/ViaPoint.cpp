#define BIORBD_API_EXPORTS
#include "Muscles/ViaPoint.h"

biorbd::muscles::ViaPoint::ViaPoint(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(v,name,parentName)
{
    m_type = "Via";
}


biorbd::muscles::ViaPoint::~ViaPoint(){

}
