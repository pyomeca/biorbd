#define BIORBD_API_EXPORTS
#include "Muscles/Compound.h"

#include "Utils/String.h"
#include "Utils/Node3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/Force.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/PathChangers.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/ForceFromOrigin.h"
#include "Muscles/ForceFromInsertion.h"

biorbd::muscles::Compound::Compound() :
    m_name(std::make_shared<biorbd::utils::String>("")),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>(biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>()),
    m_force(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Force>>>(2))
{
    (*m_force)[0] = std::make_shared<biorbd::muscles::ForceFromOrigin>();
    (*m_force)[1] = std::make_shared<biorbd::muscles::ForceFromInsertion>();
}

biorbd::muscles::Compound::Compound(const biorbd::utils::String &name) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>(biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>()),
    m_force(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Force>>>(2))
{
    (*m_force)[0] = std::make_shared<biorbd::muscles::ForceFromOrigin>();
    (*m_force)[1] = std::make_shared<biorbd::muscles::ForceFromInsertion>();
}

biorbd::muscles::Compound::Compound(
        const biorbd::utils::String &name,
        const biorbd::muscles::PathChangers& wrap) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>(biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>(wrap)),
    m_force(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Force>>>(2))
{
    (*m_force)[0] = std::make_shared<biorbd::muscles::ForceFromOrigin>();
    (*m_force)[1] = std::make_shared<biorbd::muscles::ForceFromInsertion>();
}

biorbd::muscles::Compound::Compound(
        const biorbd::muscles::Compound &muscle) :
    m_name(muscle.m_name),
    m_type(muscle.m_type),
    m_pathChanger(muscle.m_pathChanger),
    m_force(muscle.m_force)
{

}

biorbd::muscles::Compound::Compound(
        std::shared_ptr<biorbd::muscles::Compound> muscle) :
    m_name(muscle->m_name),
    m_type(muscle->m_type),
    m_pathChanger(muscle->m_pathChanger),
    m_force(muscle->m_force)
{

}

biorbd::muscles::Compound::~Compound()
{

}

void biorbd::muscles::Compound::DeepCopy(const biorbd::muscles::Compound &other)
{
    *m_name = *other.m_name;
    *m_type = *other.m_type;
    *m_pathChanger = other.m_pathChanger->DeepCopy();
    m_force->resize(other.m_force->size());
    for (unsigned int i=0; i<other.m_force->size(); ++i)
        if ( std::dynamic_pointer_cast<biorbd::muscles::ForceFromOrigin>((*other.m_force)[i]) )
            (*m_force)[i] = std::make_shared<biorbd::muscles::ForceFromOrigin>(
                    std::static_pointer_cast<biorbd::muscles::ForceFromOrigin>((*other.m_force)[i])->DeepCopy() );
        else if ( std::dynamic_pointer_cast<biorbd::muscles::ForceFromInsertion>((*other.m_force)[i]) )
            (*m_force)[i] = std::make_shared<biorbd::muscles::ForceFromInsertion>(
                    std::static_pointer_cast<biorbd::muscles::ForceFromInsertion>((*other.m_force)[i])->DeepCopy() );
        else
            (*m_force)[i] = std::make_shared<biorbd::muscles::Force>( (*other.m_force)[i]->DeepCopy() );

}

const biorbd::utils::String &biorbd::muscles::Compound::name() const
{
    return *m_name;
}

void biorbd::muscles::Compound::setName(const biorbd::utils::String &name)
{
    *m_name = name;
}

biorbd::muscles::MUSCLE_TYPE biorbd::muscles::Compound::type() const
{
    return *m_type;
}

const biorbd::muscles::PathChangers &biorbd::muscles::Compound::pathChanger() {
    return *m_pathChanger;
}

void biorbd::muscles::Compound::addPathObject(biorbd::utils::Node3d &w)  {
    m_pathChanger->addPathChanger(w);
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>>& biorbd::muscles::Compound::force() {
    return *m_force;
}

