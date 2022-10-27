#define BIORBD_API_EXPORTS
#include "InternalForces/Compound.h"

#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "InternalForces/PathModifiers.h"

using namespace BIORBD_NAMESPACE;

internalforce::Compound::Compound() :
    m_name(std::make_shared<utils::String>("")),
    m_pathChanger(std::make_shared<internalforce::PathModifiers>()),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

internalforce::Compound::Compound(const utils::String &name) :
    m_name(std::make_shared<utils::String>(name)),
    m_pathChanger(std::make_shared<internalforce::PathModifiers>()),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

internalforce::Compound::Compound(
    const utils::String &name,
    const internalforce::PathModifiers &pathModifiers) :
    m_name(std::make_shared<utils::String>(name)),
    m_pathChanger(std::make_shared<internalforce::PathModifiers>(pathModifiers)),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

internalforce::Compound::Compound(
    const internalforce::Compound &other) :
    m_name(other.m_name),
    m_pathChanger(other.m_pathChanger),
    m_force(other.m_force)
{

}

internalforce::Compound::Compound(
    std::shared_ptr<internalforce::Compound> other) :
    m_name(other->m_name),
    m_pathChanger(other->m_pathChanger),
    m_force(other->m_force)
{

}

internalforce::Compound::~Compound()
{

}

void internalforce::Compound::DeepCopy(const internalforce::Compound &other)
{
    *m_name = *other.m_name;
    *m_pathChanger = other.m_pathChanger->DeepCopy();
    *m_force = *other.m_force;
}

const utils::String &internalforce::Compound::name() const
{
    return *m_name;
}

void internalforce::Compound::setName(const utils::String &name)
{
    *m_name = name;
}

const internalforce::PathModifiers &internalforce::Compound::pathModifier()
{
    return *m_pathChanger;
}

void internalforce::Compound::addPathObject(utils::Vector3d &wrap)
{
    m_pathChanger->addPathChanger(wrap);
}

const utils::Scalar& internalforce::Compound::force()
{
    return *m_force;
}
