#define BIORBD_API_EXPORTS
#include "../include/s2mMusclePathChanger.h"

s2mMusclePathChanger::s2mMusclePathChanger(const Eigen::Vector3d &v, // Position du noeud
                                           const s2mString &name,  // Nom du noeud
                                           const s2mString &parentName) :
    s2mNodeMuscle(v,name,parentName)
{
    //ctor
}

s2mMusclePathChanger::~s2mMusclePathChanger()
{

}

const s2mString &s2mMusclePathChanger::type() const
{
    return m_type;
}

