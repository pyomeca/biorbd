#define BIORBD_API_EXPORTS
#include "s2mNodeMuscle.h"

s2mNodeMuscle::s2mNodeMuscle(
        const Eigen::Vector3d &v,
        const s2mString &name,
        const s2mString &parentName) :
    s2mNode(v, name,parentName)
{
}

s2mNodeMuscle::~s2mNodeMuscle()
{
    //dtor
}
