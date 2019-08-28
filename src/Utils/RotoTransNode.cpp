#define BIORBD_API_EXPORTS
#include "Utils/RotoTransNode.h"

#include <Eigen/Dense>
#include "Utils/String.h"

biorbd::utils::RotoTransNode::RotoTransNode() :
    biorbd::utils::RotoTrans(),
    biorbd::utils::Node()
{

}

biorbd::utils::RotoTransNode::RotoTransNode(
        const RotoTrans &rt,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::RotoTrans(rt),
    biorbd::utils::Node(name, parentName)
{

}

biorbd::utils::RotoTransNode biorbd::utils::RotoTransNode::DeepCopy() const
{
    biorbd::utils::RotoTransNode copy;
    static_cast<biorbd::utils::RotoTrans&>(copy) = this->biorbd::utils::RotoTrans::DeepCopy();
    static_cast<biorbd::utils::Node&>(copy) = this->biorbd::utils::Node::DeepCopy();
    return copy;
}

void biorbd::utils::RotoTransNode::DeepCopy(const RotoTransNode &other)
{
    biorbd::utils::RotoTrans::DeepCopy(other);
    biorbd::utils::Node::DeepCopy(other);
}

