#define BIORBD_API_EXPORTS
#include "RigidBody/NodeBone.h"

#include "Utils/Error.h"

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
    m_nbAxesToRemove(0),
    m_technical(isTechnical),
    m_anatomical(isAnatomical),
    m_id(parentID)
{
    m_axesRemoved.resize(3);
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
    m_nbAxesToRemove(0),
    m_technical(isTechnical),
    m_anatomical(isAnatomical),
    m_id(parentID)
{
    m_axesRemoved.resize(3);
    addAxesToRemove(axesToRemove);
    //ctor
}


bool biorbd::rigidbody::NodeBone::isAnatomical() const
{
    return m_anatomical;
}



bool biorbd::rigidbody::NodeBone::isTechnical() const
{
    return m_technical;
}


int biorbd::rigidbody::NodeBone::parentId() const
{
    return m_id;
}

biorbd::rigidbody::NodeBone biorbd::rigidbody::NodeBone::removeAxes() const
{
    biorbd::rigidbody::NodeBone pos(*this);
    for (unsigned int i=0; i<m_axesRemoved.size(); ++i)
        if (isAxisRemoved(i))
            pos(i) = 0;
    return pos;
}

bool biorbd::rigidbody::NodeBone::isAxisRemoved(unsigned int i) const
{
    return m_axesRemoved[i];
}

bool biorbd::rigidbody::NodeBone::isAxisKept(unsigned int i) const
{
    return !isAxisRemoved(i);
}

int biorbd::rigidbody::NodeBone::nAxesToRemove() const
{
    return m_nbAxesToRemove;
}

void biorbd::rigidbody::NodeBone::addAxesToRemove(unsigned int a)
{
    if (a>2)
        biorbd::utils::Error::error(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    m_axesRemoved[a] = true;
    ++m_nbAxesToRemove;
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
