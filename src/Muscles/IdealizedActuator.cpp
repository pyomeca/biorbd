#define BIORBD_API_EXPORTS
#include "Muscles/IdealizedActuator.h"

#include "Muscles/Caracteristics.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::IdealizedActuator::IdealizedActuator() :
    biorbd::muscles::Muscle()
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Caracteristics& caract) :
    biorbd::muscles::Muscle(name,geometry,caract)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Caracteristics& caract,
        const biorbd::muscles::PathChangers& pathChangers,
        const biorbd::muscles::StateDynamics& dynamicState) :
    biorbd::muscles::Muscle(name,geometry,caract,pathChangers,dynamicState)
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

const std::vector<biorbd::muscles::Force> &biorbd::muscles::IdealizedActuator::force(
        const biorbd::muscles::StateDynamics &emg)
{
    computeForce(emg);
    return *m_force;
}

const std::vector<biorbd::muscles::Force> &biorbd::muscles::IdealizedActuator::force(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::muscles::StateDynamics &emg,
        int)
{
    computeForce(emg);
    return *m_force;
}

const std::vector<biorbd::muscles::Force> &biorbd::muscles::IdealizedActuator::force(
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
    return caract().forceIsoMax() * (emg.activation());
}

void biorbd::muscles::IdealizedActuator::setType()
{
    *m_type = "IdealizedActuator";
}
