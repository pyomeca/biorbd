#define BIORBD_API_EXPORTS
#include "Muscles/HillTypeSimple.h"

#include "Muscles/Caracteristics.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::HillTypeSimple::HillTypeSimple() :
    biorbd::muscles::HillType()
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Caracteristics& caract) :
    biorbd::muscles::HillType(name,geometry,caract)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Caracteristics& caract,
        const biorbd::muscles::PathChangers& pathChangers,
        const biorbd::muscles::StateDynamics& dynamicState) :
    biorbd::muscles::HillType(name,geometry,caract,pathChangers,dynamicState)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const biorbd::muscles::HillType &muscle) :
    biorbd::muscles::HillType (muscle)
{

}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const std::shared_ptr<biorbd::muscles::HillType> muscle) :
    biorbd::muscles::HillType (muscle)
{

}

const std::vector<biorbd::muscles::Force> &biorbd::muscles::HillTypeSimple::force(
        const biorbd::muscles::StateDynamics &emg)
{
    // Combiner les forces
    computeForce(emg);
    return *m_force;
}

double biorbd::muscles::HillTypeSimple::multiplyCaractByActivationAndForce(
        const biorbd::muscles::StateDynamics &emg)
{
    return caract().forceIsoMax() * (emg.activation());
}

void biorbd::muscles::HillTypeSimple::setType()
{
    *m_type = "HillSimple";
}
