#define BIORBD_API_EXPORTS
#include "Muscles/WrappingNode.h"

biorbd::muscles::WrappingNode::WrappingNode(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node(v, name,parentName)
{
}

biorbd::muscles::WrappingNode::~WrappingNode()
{
    //dtor
}
