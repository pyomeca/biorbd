#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/IdealizedActuator.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/State.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::IdealizedActuator::IdealizedActuator() :
    internal_forces::muscles::Muscle()
{
    setType();
}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics) :
    internal_forces::muscles::Muscle(name,geometry,characteristics)
{
    setType();
}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg) :
    internal_forces::muscles::Muscle(name,geometry,characteristics,emg)
{
    setType();
}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::muscles::Muscle(name,geometry,characteristics, pathModifiers)
{
    setType();
}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    const internal_forces::muscles::State& emg) :
    internal_forces::muscles::Muscle(name,geometry,characteristics,pathModifiers,emg)
{
    setType();
}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(const
        internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::Muscle (other)
{

}

internal_forces::muscles::IdealizedActuator::IdealizedActuator(const
        std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::Muscle (other)
{

}

internal_forces::muscles::IdealizedActuator
internal_forces::muscles::IdealizedActuator::DeepCopy() const
{
    internal_forces::muscles::IdealizedActuator copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::IdealizedActuator::DeepCopy(const
        internal_forces::muscles::IdealizedActuator &other)
{
    internal_forces::muscles::Muscle::DeepCopy(other);
}

const utils::Scalar& internal_forces::muscles::IdealizedActuator::force(
    const internal_forces::muscles::State &emg)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& internal_forces::muscles::IdealizedActuator::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const rigidbody::GeneralizedVelocity &,
    const internal_forces::muscles::State &emg,
    bool)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& internal_forces::muscles::IdealizedActuator::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const internal_forces::muscles::State &emg,
    bool)
{
    computeForce(emg);
    return *m_force;
}

utils::Scalar
internal_forces::muscles::IdealizedActuator::getForceFromActivation(
    const internal_forces::muscles::State &emg)
{
    return characteristics().forceIsoMax() * (emg.activation());
}

void internal_forces::muscles::IdealizedActuator::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
}
