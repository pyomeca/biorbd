#define BIORBD_API_EXPORTS
#include "Utils/Node.h"

#include "Utils/String.h"

biorbd::utils::Node::Node() :
    m_name(std::make_shared<biorbd::utils::String>("")),
    m_parentName(std::make_shared<biorbd::utils::String>(""))
{

}

biorbd::utils::Node::Node(
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_parentName(std::make_shared<biorbd::utils::String>(parentName))
{

}

biorbd::utils::Node biorbd::utils::Node::DeepCopy()
{
    return biorbd::utils::Node(this->name(), this->parent());

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
