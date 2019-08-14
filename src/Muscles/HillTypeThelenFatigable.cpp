#define BIORBD_API_EXPORTS
#include "Muscles/HillTypeThelenFatigable.h"

#include "Muscles/FatigueState.h"

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &s,
        const biorbd::utils::String &dynamicFatigueType) :
    biorbd::muscles::HillTypeThelen(s),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::muscles::Geometry &g,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers &w,
        const biorbd::muscles::StateDynamics &s,
        const biorbd::utils::String &dynamicFatigueType) :
    biorbd::muscles::HillTypeThelen(g, c, w, s),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(
        const biorbd::utils::String &n,
        const biorbd::muscles::Geometry &g,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers &w,
        const biorbd::muscles::StateDynamics &s,
        const biorbd::utils::String &dynamicFatigueType) :
    biorbd::muscles::HillTypeThelen(n, g, c, w, s),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(const biorbd::muscles::Muscle &m) :
    biorbd::muscles::HillTypeThelen(m),
    biorbd::muscles::Fatigable (m)
{
    setType();
}

biorbd::muscles::HillTypeThelenFatigable::HillTypeThelenFatigable(const std::shared_ptr<biorbd::muscles::Muscle> m) :
    biorbd::muscles::HillTypeThelen(m),
    biorbd::muscles::Fatigable(m)
{
    setType();
}

void biorbd::muscles::HillTypeThelenFatigable::computeFlCE(const biorbd::muscles::StateDynamics &EMG)
{
    biorbd::muscles::HillTypeThelen::computeFlCE(EMG);   
    // Do something with m_FlCE and m_caract.fatigueParameters
    m_FlCE *= m_fatigueState->activeFibers();
}

void biorbd::muscles::HillTypeThelenFatigable::setType()
{
    m_type = "HillThelenFatigable";
}
