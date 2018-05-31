#define BIORBD_API_EXPORTS
#include "../include/s2mViaPoint.h"

s2mViaPoint::s2mViaPoint(const Eigen::Vector3d &v, // Position du noeud
                         const s2mString &name,  // Nom du noeud
                         const s2mString &parentName) :
    s2mMusclePathChanger(v,name,parentName)
{

}


s2mViaPoint::~s2mViaPoint(){

}
