#define BIORBD_API_EXPORTS
#include "../include/s2mWrappingObject.h"

s2mWrappingObject::s2mWrappingObject(const Eigen::Vector3d &v, // Position du noeud
                                     const s2mString &name,  // Nom du noeud
                                     const s2mString &parentName) :
    s2mMusclePathChanger(v,name,parentName)
{
    m_type = "Wrapping";
}

s2mWrappingObject::~s2mWrappingObject()
{
    //dtor
}

const s2mString &s2mWrappingObject::forme() const
{
    return m_forme;
}
