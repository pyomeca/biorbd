#define BIORBD_API_EXPORTS
#include "Muscles/MuscleNode.h"

s2mNodeMuscle::s2mNodeMuscle(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node(v, name,parentName)
{
}

s2mNodeMuscle::~s2mNodeMuscle()
{
    //dtor
}
