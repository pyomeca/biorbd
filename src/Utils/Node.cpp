#define BIORBD_API_EXPORTS
#include "Utils/Node.h"

#include "Utils/String.h"

biorbd::utils::Node::Node() :
    m_name(std::make_shared<biorbd::utils::String>("")),
    m_parentName(std::make_shared<biorbd::utils::String>("")),
    m_typeOfNode(std::make_shared<biorbd::utils::String>("NoType"))
{

}

biorbd::utils::Node::Node(const biorbd::utils::Node &node)
{
    // Shallow copy
    m_name = node.m_name;
    m_parentName = node.m_parentName;
    m_typeOfNode = node.m_typeOfNode;
}

biorbd::utils::Node::Node(const biorbd::utils::String &name) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_parentName(std::make_shared<biorbd::utils::String>("")),
    m_typeOfNode(std::make_shared<biorbd::utils::String>("NoType"))
{

}

biorbd::utils::Node::Node(
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_parentName(std::make_shared<biorbd::utils::String>(parentName)),
    m_typeOfNode(std::make_shared<biorbd::utils::String>("NoType"))
{

}

biorbd::utils::Node::~Node()
{

}

biorbd::utils::Node biorbd::utils::Node::DeepCopy() const
{
    biorbd::utils::Node copy;
    *copy.m_name = *m_name;
    *copy.m_parentName = *m_parentName;
    *copy.m_typeOfNode = *m_typeOfNode;
    return copy;
}

void biorbd::utils::Node::DeepCopy(
        const biorbd::utils::Node &other)
{
    m_name = std::make_shared<biorbd::utils::String>(*other.m_name);
    m_parentName = std::make_shared<biorbd::utils::String>(*other.m_parentName);
    m_typeOfNode = std::make_shared<biorbd::utils::String>(*other.m_typeOfNode);
}

const biorbd::utils::String &biorbd::utils::Node::name() const
{
    return *m_name;
}

void biorbd::utils::Node::setName(const biorbd::utils::String &name)
{
    *m_name = name;
}

const biorbd::utils::String& biorbd::utils::Node::parent() const
{
    return *m_parentName;
}

void biorbd::utils::Node::setParent(const biorbd::utils::String &parentName)
{
    *m_parentName = parentName;
}

const biorbd::utils::String &biorbd::utils::Node::typeOfNode() const
{
    return *m_typeOfNode;
}
