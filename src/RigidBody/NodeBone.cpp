#define BIORBD_API_EXPORTS
#include "RigidBody/NodeBone.h"

#include "Utils/Error.h"

s2mNodeBone::s2mNodeBone(
        const Eigen::Vector3d &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool tech,
        bool ana,
        const biorbd::utils::String& axesToRemove, // Axes à retirer
        int id) :
    biorbd::utils::Node(v, name, parentName),
    m_nbAxesToRemove(0),
    m_technical(tech),
    m_anatomical(ana),
    m_id(id)
{
    m_axesRemoved.resize(3);
    addAxesToRemove(axesToRemove);
    //ctor
}


bool s2mNodeBone::isAnatomical() const
{
    return m_anatomical;
}



bool s2mNodeBone::isTechnical() const
{
    return m_technical;
}


int s2mNodeBone::parentId() const
{
    return m_id;
}

const s2mNodeBone &s2mNodeBone::position() const
{
    return *this;
}

s2mNodeBone s2mNodeBone::position(bool removeAxes) const
{
    s2mNodeBone pos(*this);

    // S'il faut retirer des axes, les retirer
    if (removeAxes)
        for (unsigned int i=0; i<m_axesRemoved.size(); ++i)
            if (isAxisRemoved(i))
                pos(i) = 0;

    return pos;
}

bool s2mNodeBone::isAxisRemoved(unsigned int i) const
{
    return m_axesRemoved[i];
}

bool s2mNodeBone::isAxisKept(unsigned int i) const
{
    return !isAxisRemoved(i);
}

int s2mNodeBone::nAxesToRemove() const
{
    return m_nbAxesToRemove;
}

void s2mNodeBone::addAxesToRemove(unsigned int a)
{
    if (a>2)
        biorbd::utils::Error::error(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    m_axesRemoved[a] = true;
    ++m_nbAxesToRemove;
}

void s2mNodeBone::addAxesToRemove(const biorbd::utils::String& s)
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

void s2mNodeBone::addAxesToRemove(const std::vector<unsigned int>& axis)
{
    for (unsigned int i=0; i<axis.size(); ++i)
        addAxesToRemove(axis[i]);
}

void s2mNodeBone::addAxesToRemove(const std::vector<biorbd::utils::String>& axis)
{
    for (unsigned int i=0; i<axis.size(); ++i)
        addAxesToRemove(axis[i]);
}

biorbd::utils::String s2mNodeBone::axesToRemove()
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
