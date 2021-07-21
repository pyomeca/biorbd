#define BIORBD_API_EXPORTS
#include "Utils/RotoTransNode.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

utils::RotoTransNode::RotoTransNode() :
    utils::RotoTrans(),
    utils::Node()
{
    setType();
}

utils::RotoTransNode::RotoTransNode(
    const RotoTrans &rt,
    const utils::String &name,
    const utils::String &parentName) :
    utils::RotoTrans(rt),
    utils::Node(name, parentName)
{
    setType();
}

utils::RotoTransNode utils::RotoTransNode::DeepCopy() const
{
    utils::RotoTransNode copy;
    copy.DeepCopy(*this);
    return copy;
}

void utils::RotoTransNode::DeepCopy(const RotoTransNode &other)
{
    this->utils::RotoTrans::operator=(other);
    utils::Node::DeepCopy(other);
}

void utils::RotoTransNode::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::ROTOTRANS;
}

void utils::RotoTransNode::operator=(
    const utils::RotoTrans &other)
{
    *this = RotoTransNode(other, "", "");
}

utils::RotoTrans utils::RotoTransNode::operator*(
    const utils::RotoTransNode& other) const
{
    return this->utils::RotoTrans::operator*(other);
}

utils::RotoTransNode utils::operator*(
    const utils::RotoTrans &other,
    const utils::RotoTransNode &me)
{
    return utils::RotoTransNode(
               other.operator*(me),
               me.utils::Node::name(),
               me.parent());
}
