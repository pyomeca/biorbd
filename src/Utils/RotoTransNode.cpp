#define BIORBD_API_EXPORTS
#include "Utils/RotoTransNode.h"

#include <Eigen/Dense>
#include "Utils/String.h"

biorbd::utils::RotoTransNode::RotoTransNode() :
    biorbd::utils::RotoTrans(),
    biorbd::utils::Node()
{
    setType();
}

biorbd::utils::RotoTransNode::RotoTransNode(
        const RotoTrans &rt,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::RotoTrans(rt),
    biorbd::utils::Node(name, parentName)
{
    setType();
}

biorbd::utils::RotoTransNode biorbd::utils::RotoTransNode::DeepCopy() const
{
    biorbd::utils::RotoTransNode copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::RotoTransNode::DeepCopy(const RotoTransNode &other)
{
    biorbd::utils::RotoTrans::DeepCopy(other);
    biorbd::utils::Node::DeepCopy(other);
}

void biorbd::utils::RotoTransNode::setType()
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::ROTOTRANS;
}

