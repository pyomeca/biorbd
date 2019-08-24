#define BIORBD_API_EXPORTS
#include "Muscles/HillTypeThelenFatigable.h"

#include "Muscles/FatigueState.h"

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable() :
    biorbd::muscles::HillTypeThelen(),
    biorbd::muscles::Fatigable ("Simple")
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract) :
    biorbd::muscles::HillTypeThelen(name, geometry, caract),
    biorbd::muscles::Fatigable ("Simple")
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract,
        const biorbd::utils::String &dynamicFatigueType) :
    biorbd::muscles::HillTypeThelen(name, geometry, caract),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract,
        const biorbd::muscles::PathChangers &pathChangers,
        const biorbd::muscles::StateDynamics &dynamicState) :
    biorbd::muscles::HillTypeThelen(name, geometry, caract, pathChangers, dynamicState),
    biorbd::muscles::Fatigable ("Simple")
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract,
        const biorbd::muscles::PathChangers &pathChangers,
        const biorbd::muscles::StateDynamics &dynamicState,
        const biorbd::utils::String &dynamicFatigueType) :
    biorbd::muscles::HillTypeThelen(name, geometry, caract, pathChangers, dynamicState),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::muscles::Muscle &muscle) :
    biorbd::muscles::HillTypeThelen (muscle),
    biorbd::muscles::Fatigable (dynamic_cast<const biorbd::muscles::Fatigable&>(muscle))
{

}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const std::shared_ptr<biorbd::muscles::Muscle> muscle) :
    biorbd::muscles::HillTypeThelen (muscle),
    biorbd::muscles::Fatigable (std::dynamic_pointer_cast<biorbd::muscles::Fatigable>(muscle))
{

}

void biorbd::muscles::HillTypeThelenFatigable::computeFlCE(const biorbd::muscles::StateDynamics &EMG)
{
    biorbd::muscles::HillTypeThelen::computeFlCE(EMG);   
    // Do something with m_FlCE and m_caract.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void biorbd::muscles::HillTypeThelenFatigable::setType()
{
    *m_type = "HillThelenFatigable";
}
