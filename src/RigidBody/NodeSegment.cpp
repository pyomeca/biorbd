#define BIORBD_API_EXPORTS
#include "RigidBody/NodeSegment.h"

#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

rigidbody::NodeSegment::NodeSegment() :
    utils::Vector3d(0, 0, 0),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Vector3d &other
) :
    utils::Vector3d(other),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

rigidbody::NodeSegment::NodeSegment(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    int parentID) :
    utils::Vector3d(x, y, z, name, parentName),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{

}

rigidbody::NodeSegment::NodeSegment(
    const utils::Vector3d &node,
    const utils::String &name,
    const utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    int parentID) :
    utils::Vector3d(node, name, parentName),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{

}

rigidbody::NodeSegment rigidbody::NodeSegment::DeepCopy() const
{
    rigidbody::NodeSegment copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::NodeSegment::DeepCopy(
    const rigidbody::NodeSegment& other)
{
    utils::Node::DeepCopy(other);
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
    *m_id = *other.m_id;
}

void rigidbody::NodeSegment::setValues(
    const rigidbody::NodeSegment& other) 
{
    (*this)[0] = other[0];
    (*this)[1] = other[1];
    (*this)[2] = other[2];
}


bool rigidbody::NodeSegment::isAnatomical() const
{
    return *m_anatomical;
}



bool rigidbody::NodeSegment::isTechnical() const
{
    return *m_technical;
}


int rigidbody::NodeSegment::parentId() const
{
    return *m_id;
}

void rigidbody::NodeSegment::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::BONE_POINT;
}
