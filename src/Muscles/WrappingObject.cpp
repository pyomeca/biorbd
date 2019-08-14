#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

s2mWrappingObject::s2mWrappingObject(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    s2mMusclePathChanger(v,name,parentName)
{
    m_type = "Wrapping";
}

s2mWrappingObject::~s2mWrappingObject()
{
    //dtor
}

const biorbd::utils::String &s2mWrappingObject::forme() const
{
    return m_forme;
}
