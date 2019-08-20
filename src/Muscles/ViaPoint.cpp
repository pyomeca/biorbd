#define BIORBD_API_EXPORTS
#include "Muscles/ViaPoint.h"

biorbd::muscles::ViaPoint::ViaPoint(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(x, y, z, name, parentName)
{

}

biorbd::muscles::ViaPoint::ViaPoint(
        const biorbd::muscles::MuscleNode &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(v,name,parentName)
{
    m_type = "Via";
}

biorbd::muscles::ViaPoint::~ViaPoint(){

}
