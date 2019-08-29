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
    m_type(std::make_shared<biorbd::utils::String>("")),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>()),
    m_force(std::make_shared<std::vector<biorbd::muscles::Force>>())
{
    m_force->push_back(biorbd::muscles::ForceFromOrigin());
    m_force->push_back(biorbd::muscles::ForceFromInsertion());
}

biorbd::muscles::Compound::Compound(const biorbd::utils::String &name) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::utils::String>("")),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>()),
    m_force(std::make_shared<std::vector<biorbd::muscles::Force>>())
{
    m_force->push_back(biorbd::muscles::ForceFromOrigin());
    m_force->push_back(biorbd::muscles::ForceFromInsertion());
}

biorbd::muscles::Compound::Compound(
        const biorbd::utils::String &name,
        const biorbd::muscles::PathChangers& wrap) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::utils::String>("")),
    m_pathChanger(std::make_shared<biorbd::muscles::PathChangers>(wrap)),
    m_force(std::make_shared<std::vector<biorbd::muscles::Force>>())
{
    m_force->push_back(biorbd::muscles::ForceFromOrigin());
    m_force->push_back(biorbd::muscles::ForceFromInsertion());
}

biorbd::muscles::Compound::Compound(
        const biorbd::muscles::Compound &muscle)
{
    m_name = muscle.m_name;
    m_type = muscle.m_type;
    m_pathChanger = muscle.m_pathChanger;
    m_force = muscle.m_force;
}

biorbd::muscles::Compound::Compound(
        std::shared_ptr<biorbd::muscles::Compound> muscle)
{
    m_name = muscle->m_name;
    m_type = muscle->m_type;
    m_pathChanger = muscle->m_pathChanger;
    m_force = muscle->m_force;
}

biorbd::muscles::Compound::~Compound()
{

}

const biorbd::utils::String &biorbd::muscles::Compound::name() const
{
    return *m_name;
}

void biorbd::muscles::Compound::setName(const biorbd::utils::String &name)
{
    *m_name = name;
}

const biorbd::utils::String &biorbd::muscles::Compound::type() const
{
    return *m_type;
}

const biorbd::muscles::PathChangers &biorbd::muscles::Compound::pathChanger() {
    return *m_pathChanger;
}

void biorbd::muscles::Compound::addPathObject(biorbd::utils::Node3d &w)  {
    m_pathChanger->addPathChanger(w);
}

const std::vector<biorbd::muscles::Force>& biorbd::muscles::Compound::force() {
    return *m_force;
}

void biorbd::muscles::Compound::copyForce(const std::shared_ptr<std::vector<Force> > &force)
{
    m_force = force;
}

