#define BIORBD_API_EXPORTS
#include "Muscles/PathChanger.h"

s2mMusclePathChanger::s2mMusclePathChanger(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    s2mNodeMuscle(v,name,parentName)
{
    //ctor
}

s2mMusclePathChanger::~s2mMusclePathChanger()
{

}

const biorbd::utils::String &s2mMusclePathChanger::type() const
{
    return m_type;
}

