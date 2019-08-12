#define BIORBD_API_EXPORTS
#include "s2mNodeBone.h"

#include "Utils/Error.h"

s2mNodeBone::s2mNodeBone(
        const Eigen::Vector3d &v,
        const s2mString &name,
        const s2mString &parentName,
        bool tech,
        bool ana,
        const s2mString& axesToRemove, // Axes à retirer
        int id) :
    s2mNode(v, name, parentName),
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
        s2mError::s2mAssert(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    m_axesRemoved[a] = true;
    ++m_nbAxesToRemove;
}

void s2mNodeBone::addAxesToRemove(s2mString s)
{
    for (unsigned int i=0; i<s.length(); ++i)
        if (!s(i).compare("x"))
            addAxesToRemove(0);
        else if (!s(i).compare("y"))
            addAxesToRemove(1);
        else if (!s(i).compare("z"))
            addAxesToRemove(2);
        else
            s2mError::s2mAssert(false, "Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
}

void s2mNodeBone::addAxesToRemove(std::vector<unsigned int> a)
{
    for (std::vector<unsigned int>::iterator it=a.begin(); it!=a.end(); ++it)
        addAxesToRemove(*it);
}

void s2mNodeBone::addAxesToRemove(std::vector<s2mString> a)
{
    for (std::vector<s2mString>::iterator it=a.begin(); it!=a.end(); ++it)
        addAxesToRemove(*it);
}

s2mString s2mNodeBone::axesToRemove()
{
    s2mString axes;
    if (isAxisRemoved(0))
        axes += "x";
    if (isAxisRemoved(1))
        axes += "y";
    if (isAxisRemoved(2))
        axes += "z";
    return axes;
}
