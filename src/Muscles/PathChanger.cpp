#define BIORBD_API_EXPORTS
#include "Muscles/PathChanger.h"

biorbd::muscles::PathChanger::PathChanger(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::MuscleNode(x, y, z, name, parentName)
{

}

biorbd::muscles::PathChanger::PathChanger(
        const biorbd::muscles::MuscleNode &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::MuscleNode(v, name, parentName)
{
    //ctor
}

biorbd::muscles::PathChanger::~PathChanger()
{

}


const biorbd::utils::String &biorbd::muscles::PathChanger::type() const
{
    return m_type;
}

