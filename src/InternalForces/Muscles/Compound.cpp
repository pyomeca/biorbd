#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/Compound.h"

#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/Muscles/State.h"

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

muscles::Compound::Compound() :
    m_name(std::make_shared<utils::String>("")),
    m_type(std::make_shared<muscles::MUSCLE_TYPE>
           (muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)&&std::make_shared<utils::String>("")),
    m_pathChanger(std::make_shared<PathModifiers>()),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

muscles::Compound::Compound(const utils::String &name) :
    m_name(std::make_shared<utils::String>(name)),
    m_type(std::make_shared<muscles::MUSCLE_TYPE>
           (muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<PathModifiers>()),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

muscles::Compound::Compound(
    const utils::String &name,
    const PathModifiers &pathModifiers) :
    m_name(std::make_shared<utils::String>(name)),
    m_type(std::make_shared<muscles::MUSCLE_TYPE>
           (muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<PathModifiers>(pathModifiers)),
    m_force(std::make_shared<utils::Scalar>(0))
{

}

muscles::Compound::Compound(
    const muscles::Compound &other) :
    m_name(other.m_name),
    m_type(other.m_type),
    m_pathChanger(other.m_pathChanger),
    m_force(other.m_force)
{

}

muscles::Compound::Compound(
    std::shared_ptr<muscles::Compound> other) :
    m_name(other->m_name),
    m_type(other->m_type),
    m_pathChanger(other->m_pathChanger),
    m_force(other->m_force)
{

}

muscles::Compound::~Compound()
{

}

void muscles::Compound::DeepCopy(const muscles::Compound &other)
{
    *m_name = *other.m_name;
    *m_type = *other.m_type;
    *m_pathChanger = other.m_pathChanger->DeepCopy();
    *m_force = *other.m_force;
}

const utils::String &muscles::Compound::name() const
{
    return *m_name;
}

void muscles::Compound::setName(const utils::String &name)
{
    *m_name = name;
}

muscles::MUSCLE_TYPE muscles::Compound::type() const
{
    return *m_type;
}

const PathModifiers &muscles::Compound::pathModifier()
{
    return *m_pathChanger;
}

void muscles::Compound::addPathObject(utils::Vector3d &wrap)
{
    m_pathChanger->addPathChanger(wrap);
}

const utils::Scalar& muscles::Compound::force()
{
    return *m_force;
}
