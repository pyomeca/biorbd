#define BIORBD_API_EXPORTS
#include "Muscles/PathChanger.h"

biorbd::muscles::PathChanger::PathChanger(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::MuscleNode(v,name,parentName)
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

