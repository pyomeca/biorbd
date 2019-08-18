#define BIORBD_API_EXPORTS
#include "Muscles/MuscleNode.h"

biorbd::muscles::MuscleNode::MuscleNode(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node(v, name,parentName)
{
}

biorbd::muscles::MuscleNode::~MuscleNode()
{
    //dtor
}
