#define BIORBD_API_EXPORTS
#include "Muscles/IdealizedActuator.h"

#include "Utils/String.h"
#include "Muscles/Characteristics.h"
#include "Muscles/State.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::IdealizedActuator::IdealizedActuator() :
    muscles::Muscle()
{
    setType();
}

muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics) :
    muscles::Muscle(name,geometry,characteristics)
{
    setType();
}

muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg) :
    muscles::Muscle(name,geometry,characteristics,emg)
{
    setType();
}

muscles::IdealizedActuator::IdealizedActuator(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::Muscle(name,geometry,characteristics, pathModifiers)
{
    setType();
}

muscles::IdealizedActuator::IdealizedActuator(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& emg) :
    muscles::Muscle(name,geometry,characteristics,pathModifiers,emg)
{
    setType();
}

muscles::IdealizedActuator::IdealizedActuator(const
        muscles::Muscle &other) :
    muscles::Muscle (other)
{

}

muscles::IdealizedActuator::IdealizedActuator(const
        std::shared_ptr<muscles::Muscle> other) :
    muscles::Muscle (other)
{

}

muscles::IdealizedActuator
muscles::IdealizedActuator::DeepCopy() const
{
    muscles::IdealizedActuator copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::IdealizedActuator::DeepCopy(const
        muscles::IdealizedActuator &other)
{
    muscles::Muscle::DeepCopy(other);
}

const utils::Scalar& muscles::IdealizedActuator::force(
    const muscles::State &emg)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& muscles::IdealizedActuator::force(
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const rigidbody::GeneralizedVelocity &,
    const muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& muscles::IdealizedActuator::force(
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

utils::Scalar
muscles::IdealizedActuator::getForceFromActivation(
    const muscles::State &emg)
{
    return characteristics().forceIsoMax() * (emg.activation());
}

void muscles::IdealizedActuator::setType()
{
    *m_type = muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
}
