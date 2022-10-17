#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/IdealizedActuator.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/State.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::IdealizedActuator::IdealizedActuator() :
    internalforce::muscles::Muscle()
{
    setType();
}

internalforce::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics) :
    internalforce::muscles::Muscle(name,geometry,characteristics)
{
    setType();
}

internalforce::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg) :
    internalforce::muscles::Muscle(name,geometry,characteristics,emg)
{
    setType();
}

internalforce::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const PathModifiers &pathModifiers) :
    internalforce::muscles::Muscle(name,geometry,characteristics, pathModifiers)
{
    setType();
}

internalforce::muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const PathModifiers &pathModifiers,
    const internalforce::muscles::State& emg) :
    internalforce::muscles::Muscle(name,geometry,characteristics,pathModifiers,emg)
{
    setType();
}

internalforce::muscles::IdealizedActuator::IdealizedActuator(const
        internalforce::muscles::Muscle &other) :
    internalforce::muscles::Muscle (other)
{

}

internalforce::muscles::IdealizedActuator::IdealizedActuator(const
        std::shared_ptr<internalforce::muscles::Muscle> other) :
    internalforce::muscles::Muscle (other)
{

}

internalforce::muscles::IdealizedActuator
internalforce::muscles::IdealizedActuator::DeepCopy() const
{
    internalforce::muscles::IdealizedActuator copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::IdealizedActuator::DeepCopy(const
        internalforce::muscles::IdealizedActuator &other)
{
    internalforce::muscles::Muscle::DeepCopy(other);
}

const utils::Scalar& internalforce::muscles::IdealizedActuator::force(
    const internalforce::muscles::State &emg)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& internalforce::muscles::IdealizedActuator::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const rigidbody::GeneralizedVelocity &,
    const internalforce::muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& internalforce::muscles::IdealizedActuator::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const internalforce::muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

utils::Scalar
internalforce::muscles::IdealizedActuator::getForceFromActivation(
    const internalforce::muscles::State &emg)
{
    return characteristics().forceIsoMax() * (emg.activation());
}

void internalforce::muscles::IdealizedActuator::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
}
