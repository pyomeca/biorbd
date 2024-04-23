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
#include "InternalForces/Ligaments/LigamentSpringSecondOrder.h"

using namespace BIORBD_NAMESPACE;
internal_forces::ligaments::LigamentSpringSecondOrder::LigamentSpringSecondOrder() :
    internal_forces::ligaments::Ligament(),
    m_stiffness(std::make_shared<utils::Scalar>()),
    m_epsilon(std::make_shared<utils::Scalar>(1))
{
    setType();

}

internal_forces::ligaments::LigamentSpringSecondOrder::LigamentSpringSecondOrder(
    const utils::Scalar &stiffness,
    const utils::Scalar &epsilon,
    const utils::String & name,
    const internal_forces::Geometry & position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics):
    internal_forces::ligaments::Ligament(name,position,characteristics),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_epsilon(std::make_shared<utils::Scalar>(epsilon))
{
    setType();
}

internal_forces::ligaments::LigamentSpringSecondOrder::LigamentSpringSecondOrder(
    const utils::Scalar &stiffness,
    const utils::Scalar &epsilon,
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers):
    internal_forces::ligaments::Ligament(name,position,characteristics,pathModifiers),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_epsilon(std::make_shared<utils::Scalar>(epsilon))
{
    setType();
}

internal_forces::ligaments::LigamentSpringSecondOrder
internal_forces::ligaments::LigamentSpringSecondOrder::DeepCopy() const
{
    internal_forces::ligaments::LigamentSpringSecondOrder copy;
    copy.DeepCopy(*this);
    return copy;
}
void internal_forces::ligaments::LigamentSpringSecondOrder::DeepCopy(
        const internal_forces::ligaments::LigamentSpringSecondOrder &other)
{
    internal_forces::ligaments::Ligament::DeepCopy(other);
    *m_stiffness = *other.m_stiffness;
    *m_epsilon = *other.m_epsilon;
}

const utils::Scalar& internal_forces::ligaments::LigamentSpringSecondOrder::stiffness() const
{
    return *m_stiffness;
}

void internal_forces::ligaments::LigamentSpringSecondOrder::setStiffness(
    const utils::Scalar &val)
{
    *m_stiffness = val;
}

const utils::Scalar& internal_forces::ligaments::LigamentSpringSecondOrder::epsilon() const
{
    return *m_epsilon;
}

void internal_forces::ligaments::LigamentSpringSecondOrder::setEpsilon(
    const utils::Scalar &val)
{
    *m_epsilon = val;
}

internal_forces::ligaments::LigamentSpringSecondOrder::LigamentSpringSecondOrder(
    const internal_forces::ligaments::Ligament &other) :
    internal_forces::ligaments::Ligament (other)
{
    const internal_forces::ligaments::LigamentSpringSecondOrder & ligament_tp(
        dynamic_cast<const internal_forces::ligaments::LigamentSpringSecondOrder &>(other));
    m_stiffness = ligament_tp.m_stiffness;
    m_epsilon = ligament_tp.m_epsilon;
}


internal_forces::ligaments::LigamentSpringSecondOrder::LigamentSpringSecondOrder(
    const std::shared_ptr<internal_forces::ligaments::Ligament> other) :
    internal_forces::ligaments::Ligament (other)
{
    const std::shared_ptr<internal_forces::ligaments::LigamentSpringSecondOrder> ligament_tp(
        std::dynamic_pointer_cast<internal_forces::ligaments::LigamentSpringSecondOrder>(other));
    utils::Error::check(ligament_tp != nullptr, "ligament must be of a spring second order Type");
    m_stiffness = ligament_tp->m_stiffness;
    m_epsilon = ligament_tp->m_epsilon;
}

internal_forces::ligaments::LigamentSpringSecondOrder::~LigamentSpringSecondOrder()
{

}

void internal_forces::ligaments::LigamentSpringSecondOrder::setType()
{
    *m_type = internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_SECOND_ORDER;
}

void internal_forces::ligaments::LigamentSpringSecondOrder::computeFl()
{
    utils::Scalar d = position().length() - characteristics().ligamentSlackLength();
    *m_Fl = (*m_stiffness/2) * (d + std::sqrt(d*d + *m_epsilon**m_epsilon));
}
