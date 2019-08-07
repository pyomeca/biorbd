#define BIORBD_API_EXPORTS
#include "s2mMuscleCompound.h"

s2mMuscleCompound::s2mMuscleCompound(const s2mString &name, const s2mMusclePathChangers& wrap) :
    m_pathChanger(wrap),
    m_name(name)
{
    this->m_force.clear();
    //ctor
}

s2mMuscleCompound::s2mMuscleCompound(const s2mMuscleCompound &m)
{
    this->m_pathChanger = m.m_pathChanger;
    this->copyForce(m.m_force);
    this->m_type = m.m_type;
    this->m_name = m.m_name;
}

s2mMuscleCompound::~s2mMuscleCompound()
{
    //dtor
}

const s2mMusclePathChangers &s2mMuscleCompound::pathChanger() {
    return m_pathChanger;
}

void s2mMuscleCompound::addPathObject(s2mMusclePathChanger &w)  {
    m_pathChanger.addPathChanger(w);
}

const s2mString &s2mMuscleCompound::type() const {return m_type;}

const std::vector<std::shared_ptr<s2mMuscleForce>>& s2mMuscleCompound::force() {
    return m_force;
}

const s2mString &s2mMuscleCompound::name() const {
    return m_name;
}

void s2mMuscleCompound::setName(const s2mString &name) {
    m_name = name;
}

void s2mMuscleCompound::copyForce(const std::vector<std::shared_ptr<s2mMuscleForce>> &force)
{
    m_force = force;
}

