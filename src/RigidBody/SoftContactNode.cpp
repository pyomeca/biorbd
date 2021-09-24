#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContactNode.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContactNode::SoftContactNode() :
    rigidbody::NodeSegment()
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z) :
    rigidbody::NodeSegment(x, y, z)
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Vector3d &other) :
    rigidbody::NodeSegment(other)
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z,
        const utils::String &name,
        const utils::String &parentName,
        int parentID) :
    rigidbody::NodeSegment(x, y, z, name, parentName, parentID)
{

}

rigidbody::SoftContactNode::SoftContactNode(
        const utils::Vector3d &node,
        const utils::String &name,
        const utils::String &parentName,
        int parentID) :
    rigidbody::NodeSegment(node, name, parentName, parentID)
{

}

rigidbody::SoftContactNode rigidbody::SoftContactNode::DeepCopy() const
{

}

void rigidbody::SoftContactNode::DeepCopy(
        const rigidbody::SoftContactNode &other)
{

}

int rigidbody::SoftContactNode::parentId() const
{

}

void rigidbody::SoftContactNode::setType()
{

}
