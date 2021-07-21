#define BIORBD_API_EXPORTS
#include "Utils/Node.h"

#include "Utils/String.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

utils::Node::Node() :
    m_name(std::make_shared<utils::String>("")),
    m_parentName(std::make_shared<utils::String>("")),
    m_typeOfNode(std::make_shared<utils::NODE_TYPE>
                 (utils::NODE_TYPE::NO_NODE_TYPE))
{

}

utils::Node::Node(
    const utils::Node &other)
{
    // Shallow copy
    m_name = other.m_name;
    m_parentName = other.m_parentName;
    m_typeOfNode = other.m_typeOfNode;
}

utils::Node::Node(const utils::String &name) :
    m_name(std::make_shared<utils::String>(name)),
    m_parentName(std::make_shared<utils::String>("")),
    m_typeOfNode(std::make_shared<utils::NODE_TYPE>
                 (utils::NODE_TYPE::NO_NODE_TYPE))
{

}

utils::Node::Node(
    const utils::String &name,
    const utils::String &parentName) :
    m_name(std::make_shared<utils::String>(name)),
    m_parentName(std::make_shared<utils::String>(parentName)),
    m_typeOfNode(std::make_shared<utils::NODE_TYPE>
                 (utils::NODE_TYPE::NO_NODE_TYPE))
{

}

utils::Node::~Node()
{

}

void utils::Node::DeepCopy(
    const utils::Node &other)
{
    *m_name = *other.m_name;
    *m_parentName = *other.m_parentName;
    *m_typeOfNode = *other.m_typeOfNode;
}

void utils::Node::setName(const utils::String &name)
{
    *m_name = name;
}

const utils::String &utils::Node::name() const
{
    return *m_name;
}

const utils::String& utils::Node::parent() const
{
    return *m_parentName;
}

void utils::Node::setParent(
    const utils::String &name)
{
    *m_parentName = name;
}

utils::NODE_TYPE utils::Node::typeOfNode() const
{
    return *m_typeOfNode;
}
