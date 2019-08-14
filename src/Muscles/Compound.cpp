#define BIORBD_API_EXPORTS
#include "Muscles/Compound.h"

biorbd::muscles::Compound::Compound(
        const biorbd::utils::String &name,
        const biorbd::muscles::PathChangers& wrap) :
    m_pathChanger(wrap),
    m_name(name)
{
    this->m_force.clear();
    //ctor
}

biorbd::muscles::Compound::Compound(const biorbd::muscles::Compound &m)
{
    this->m_pathChanger = m.m_pathChanger;
    this->copyForce(m.m_force);
    this->m_type = m.m_type;
    this->m_name = m.m_name;
}

biorbd::muscles::Compound::~Compound()
{
    //dtor
}

const biorbd::muscles::PathChangers &biorbd::muscles::Compound::pathChanger() {
    return m_pathChanger;
}

void biorbd::muscles::Compound::addPathObject(biorbd::muscles::PathChanger &w)  {
    m_pathChanger.addPathChanger(w);
}

const biorbd::utils::String &biorbd::muscles::Compound::type() const
{
    return m_type;
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>>& biorbd::muscles::Compound::force() {
    return m_force;
}

const biorbd::utils::String &biorbd::muscles::Compound::name() const {
    return m_name;
}

void biorbd::muscles::Compound::setName(const biorbd::utils::String &name) {
    m_name = name;
}

void biorbd::muscles::Compound::copyForce(const std::vector<std::shared_ptr<biorbd::muscles::Force>> &force)
{
    m_force = force;
}

