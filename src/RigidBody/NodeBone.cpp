#define BIORBD_API_EXPORTS
#include "RigidBody/NodeBone.h"

#include "Utils/Error.h"

biorbd::rigidbody::NodeBone::NodeBone() :
    biorbd::utils::Node3d(0, 0, 0),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeBone::NodeBone(double x, double y, double z) :
    biorbd::utils::Node3d(x, y, z),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeBone::NodeBone(const biorbd::utils::Node3d &other) :
    biorbd::utils::Node3d(other),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeBone::NodeBone(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool isTechnical,
        bool isAnatomical,
        const biorbd::utils::String &axesToRemove,
        int parentID) :
    biorbd::utils::Node3d(x, y, z, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
}

biorbd::rigidbody::NodeBone::NodeBone(
        const biorbd::utils::Node3d &node,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool isTechnical,
        bool isAnatomical,
        const biorbd::utils::String& axesToRemove, // Axes à retirer
        int parentID) :
    biorbd::utils::Node3d(node, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
    //ctor
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::NodeBone::DeepCopy() const
{
    biorbd::rigidbody::NodeBone copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::NodeBone::DeepCopy(const biorbd::rigidbody::NodeBone &other)
{
    biorbd::utils::Node::DeepCopy(other);
    *m_axesRemoved = *other.m_axesRemoved;
    *m_nbAxesToRemove = *other.m_nbAxesToRemove;
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
    *m_id = *other.m_id;
}


bool biorbd::rigidbody::NodeBone::isAnatomical() const
{
    return *m_anatomical;
}



bool biorbd::rigidbody::NodeBone::isTechnical() const
{
    return *m_technical;
}


int biorbd::rigidbody::NodeBone::parentId() const
{
    return *m_id;
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::NodeBone::removeAxes() const
{
    biorbd::rigidbody::NodeBone pos(*this);
    for (unsigned int i=0; i<m_axesRemoved->size(); ++i)
        if (isAxisRemoved(i))
            pos(i) = 0;
    return pos;
}

bool biorbd::rigidbody::NodeBone::isAxisRemoved(unsigned int i) const
{
    return (*m_axesRemoved)[i];
}

bool biorbd::rigidbody::NodeBone::isAxisKept(unsigned int i) const
{
    return !isAxisRemoved(i);
}

int biorbd::rigidbody::NodeBone::nAxesToRemove() const
{
    return *m_nbAxesToRemove;
}

void biorbd::rigidbody::NodeBone::setType()
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::BONE_POINT;
}

void biorbd::rigidbody::NodeBone::addAxesToRemove(unsigned int a)
{
    if (a>2)
        biorbd::utils::Error::error(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    (*m_axesRemoved)[a] = true;
    ++*m_nbAxesToRemove;
}

void biorbd::rigidbody::NodeBone::addAxesToRemove(const biorbd::utils::String& s)
{
    for (unsigned int i=0; i<s.length(); ++i)
        if (!s(i).compare("x"))
            addAxesToRemove(0);
        else if (!s(i).compare("y"))
            addAxesToRemove(1);
        else if (!s(i).compare("z"))
            addAxesToRemove(2);
        else
            biorbd::utils::Error::error(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
}

void biorbd::rigidbody::NodeBone::addAxesToRemove(const std::vector<unsigned int>& axis)
{
    for (unsigned int i=0; i<axis.size(); ++i)
        addAxesToRemove(axis[i]);
}

void biorbd::rigidbody::NodeBone::addAxesToRemove(const std::vector<biorbd::utils::String>& axis)
{
    for (unsigned int i=0; i<axis.size(); ++i)
        addAxesToRemove(axis[i]);
}

biorbd::utils::String biorbd::rigidbody::NodeBone::axesToRemove()
{
    biorbd::utils::String axes;
    if (isAxisRemoved(0))
        axes += "x";
    if (isAxisRemoved(1))
        axes += "y";
    if (isAxisRemoved(2))
        axes += "z";
    return axes;
}
