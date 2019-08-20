#define BIORBD_API_EXPORTS
#include "Muscles/WrappingNode.h"

biorbd::muscles::WrappingNode::WrappingNode(
        double x,
        double y,
        double z,
        const biorbd::utils::String &nodeName,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(x, y, z, nodeName, parentName)
{

}

biorbd::muscles::WrappingNode::WrappingNode(
        const biorbd::utils::Node &v,
        const biorbd::utils::String &nodeName,
        const biorbd::utils::String &parentName) :
    biorbd::muscles::PathChanger(v, nodeName, parentName)
{
}

biorbd::muscles::WrappingNode::~WrappingNode()
{
    //dtor
}
