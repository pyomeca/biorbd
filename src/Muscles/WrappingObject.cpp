#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

biorbd::muscles::WrappingObject::WrappingObject(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(x, y, z, name, parentName)
{

}

biorbd::muscles::WrappingObject::WrappingObject(
        const biorbd::utils::Node &v, // Position du noeud
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
