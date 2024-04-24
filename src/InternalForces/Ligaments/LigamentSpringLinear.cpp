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
#include "InternalForces/Ligaments/LigamentSpringLinear.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;
internal_forces::ligaments::LigamentSpringLinear::LigamentSpringLinear() :
    internal_forces::ligaments::Ligament(),
    m_stiffness(std::make_shared<utils::Scalar>())
{
    setType();

}

internal_forces::ligaments::LigamentSpringLinear::LigamentSpringLinear(
    const utils::Scalar &stiffness,
    const utils::String & name,
    const internal_forces::Geometry & position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics) :
    internal_forces::ligaments::Ligament(name,position,characteristics),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness))
{
    setType();
}

internal_forces::ligaments::LigamentSpringLinear::LigamentSpringLinear(
    const utils::Scalar &stiffness,
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::ligaments::Ligament(name,position,characteristics,pathModifiers),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness))
{
    setType();
}

internal_forces::ligaments::LigamentSpringLinear::LigamentSpringLinear(
    const std::shared_ptr<internal_forces::ligaments::Ligament> other) :
    internal_forces::ligaments::Ligament (other)
{
    const std::shared_ptr<internal_forces::ligaments::LigamentSpringLinear> ligament_tp(
        std::dynamic_pointer_cast<internal_forces::ligaments::LigamentSpringLinear>(other));
    utils::Error::check(ligament_tp != nullptr, "ligament must be of a spring linear Type");
    m_stiffness = ligament_tp->m_stiffness;
}

internal_forces::ligaments::LigamentSpringLinear
internal_forces::ligaments::LigamentSpringLinear::DeepCopy() const
{
    internal_forces::ligaments::LigamentSpringLinear copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ligaments::LigamentSpringLinear::DeepCopy(
        const internal_forces::ligaments::LigamentSpringLinear &other)
{
    internal_forces::ligaments::Ligament::DeepCopy(other);
    *m_stiffness = *other.m_stiffness;
}

const utils::Scalar& internal_forces::ligaments::LigamentSpringLinear::stiffness() const
{
    return *m_stiffness;
}

void internal_forces::ligaments::LigamentSpringLinear::setStiffness(
    const utils::Scalar &val)
{
    *m_stiffness = val;
}

internal_forces::ligaments::LigamentSpringLinear::LigamentSpringLinear(
    const internal_forces::ligaments::Ligament &other) :
    internal_forces::ligaments::Ligament (other)
{
    const internal_forces::ligaments::LigamentSpringLinear & ligament_tp(
        dynamic_cast<const internal_forces::ligaments::LigamentSpringLinear &>(other));
    m_stiffness = ligament_tp.m_stiffness;
}

internal_forces::ligaments::LigamentSpringLinear::~LigamentSpringLinear()
{

}

void internal_forces::ligaments::LigamentSpringLinear::setType()
{
    *m_type = internal_forces::ligaments::LIGAMENT_TYPE::LIGAMENT_SPRING_LINEAR;
}

void internal_forces::ligaments::LigamentSpringLinear::computeFl()
{
#ifdef BIORBD_USE_CASADI_MATH
    *m_Fl = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(position().length(), characteristics().ligamentSlackLength()),
                  *m_stiffness * (position().length() - characteristics().ligamentSlackLength()));
#else
    if (position().length() > characteristics().ligamentSlackLength()) {
        *m_Fl = *m_stiffness * (position().length() - characteristics().ligamentSlackLength());
    } else {
        *m_Fl = 0;
    }
#endif
}
