#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

biorbd::muscles::WrappingObject::WrappingObject(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(v,name,parentName)
{
    m_type = "Wrapping";
}

biorbd::muscles::WrappingObject::~WrappingObject()
{
    //dtor
}

const biorbd::utils::String &biorbd::muscles::WrappingObject::forme() const
{
    return m_forme;
}
