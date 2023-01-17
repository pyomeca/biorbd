#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Geometry.h"
#include "InternalForces/Ligaments/LigamentsEnums.h"
#include "InternalForces/Ligaments/LigamentCharacteristics.h"
#include "InternalForces/Ligaments/LigamentConstant.h"

using namespace BIORBD_NAMESPACE;
internal_forces::ligaments::LigamentConstant::LigamentConstant() :
    internal_forces::ligaments::Ligament(),
    m_force(std::make_shared<utils::Scalar>())
{
    setType();
}

internal_forces::ligaments::LigamentConstant::LigamentConstant(
    const utils::Scalar &force,
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics) :
    internal_forces::ligaments::Ligament(name,position,characteristics),
    m_force(std::make_shared<utils::Scalar>(force))
{
    setType();
}

internal_forces::ligaments::LigamentConstant::LigamentConstant(
    const utils::Scalar &force,
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::ligaments::Ligament(name,position,characteristics,pathModifiers),
    m_force(std::make_shared<utils::Scalar>(force))
{
    setType();
}

internal_forces::ligaments::LigamentConstant::LigamentConstant(
    const std::shared_ptr<internal_forces::ligaments::Ligament> other) :
    internal_forces::ligaments::Ligament (other)
{
    const std::shared_ptr<internal_forces::ligaments::LigamentConstant> ligament_tp(
        std::dynamic_pointer_cast<internal_forces::ligaments::LigamentConstant>(other));
    utils::Error::check(ligament_tp != nullptr, "ligament must be of a constant Type");
    m_force = ligament_tp->m_force;
}

internal_forces::ligaments::LigamentConstant::LigamentConstant(
    const internal_forces::ligaments::Ligament &other) :
    internal_forces::ligaments::Ligament (other)
{
    const internal_forces::ligaments::LigamentConstant &ligament_tp(
        dynamic_cast<const internal_forces::ligaments::LigamentConstant &>(other));
    m_force = ligament_tp.m_force;
}

internal_forces::ligaments::LigamentConstant
internal_forces::ligaments::LigamentConstant::DeepCopy() const
{
    internal_forces::ligaments::LigamentConstant copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ligaments::LigamentConstant::DeepCopy(
        const internal_forces::ligaments::LigamentConstant &other)
{
    internal_forces::ligaments::Ligament::DeepCopy(other);
    *m_force = *other.m_force;
}


internal_forces::ligaments::LigamentConstant::~LigamentConstant()
{

}

void internal_forces::ligaments::LigamentConstant::setType()
{
    *m_type = internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_CONSTANT;
}

void internal_forces::ligaments::LigamentConstant::computeFl()
{
    *m_Fl = *m_force;
}
