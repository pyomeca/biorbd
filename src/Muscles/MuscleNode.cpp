#define BIORBD_API_EXPORTS
#include "Muscles/MuscleNode.h"

biorbd::muscles::MuscleNode::MuscleNode(
        double x,
        double y,
        double z,
        const biorbd::utils::String &nodeName,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node3d(x, y, z, nodeName,parentName)
{

}

biorbd::muscles::MuscleNode::MuscleNode(
        const biorbd::utils::Node3d &v,
        const biorbd::utils::String &nodeName,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node3d(v, nodeName,parentName)
{
}

biorbd::muscles::MuscleNode::~MuscleNode()
{
    //dtor
}
