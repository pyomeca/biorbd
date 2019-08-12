#define BIORBD_API_EXPORTS
#include "s2mViaPoint.h"

s2mViaPoint::s2mViaPoint(
        const Eigen::Vector3d &v, // Position du noeud
        const biorbd::utils::String &name,  // Nom du noeud
        const biorbd::utils::String &parentName) :
    s2mMusclePathChanger(v,name,parentName)
{
    m_type = "Via";
}


s2mViaPoint::~s2mViaPoint(){

}
