#define BIORBD_API_EXPORTS
#include "Muscles/IdealizedActuator.h"

#include "Utils/String.h"
#include "Muscles/Characteristics.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::IdealizedActuator::IdealizedActuator() :
    biorbd::muscles::Muscle()
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics) :
    biorbd::muscles::Muscle(name,geometry,characteristics)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        const biorbd::muscles::StateDynamics &dynamicState) :
    biorbd::muscles::Muscle(name,geometry,characteristics,dynamicState)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        const biorbd::muscles::PathChangers &pathChangers) :
    biorbd::muscles::Muscle(name,geometry,characteristics, pathChangers)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathChangers& pathChangers,
        const biorbd::muscles::StateDynamics& dynamicState) :
    biorbd::muscles::Muscle(name,geometry,characteristics,pathChangers,dynamicState)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::muscles::Muscle &muscle) :
    biorbd::muscles::Muscle (muscle)
{

}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const std::shared_ptr<biorbd::muscles::Muscle> muscle) :
    biorbd::muscles::Muscle (muscle)
{

}

biorbd::muscles::IdealizedActuator biorbd::muscles::IdealizedActuator::DeepCopy() const
{
    biorbd::muscles::IdealizedActuator copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::IdealizedActuator::DeepCopy(const biorbd::muscles::IdealizedActuator &other)
{
    biorbd::muscles::Muscle::DeepCopy(other);
}

const std::vector<std::shared_ptr<biorbd::muscles::Force> > &biorbd::muscles::IdealizedActuator::force(
        const biorbd::muscles::StateDynamics &emg)
{
    computeForce(emg);
    return *m_force;
}

const std::vector<std::shared_ptr<biorbd::muscles::Force> > &biorbd::muscles::IdealizedActuator::force(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::muscles::StateDynamics &emg,
        int)
{
    computeForce(emg);
    return *m_force;
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>> &biorbd::muscles::IdealizedActuator::force(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::muscles::StateDynamics &emg,
        int)
{
    computeForce(emg);
    return *m_force;
}

double biorbd::muscles::IdealizedActuator::getForceFromActivation(
        const biorbd::muscles::State &emg)
{
    return characteristics().forceIsoMax() * (emg.activation());
}

void biorbd::muscles::IdealizedActuator::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
}
