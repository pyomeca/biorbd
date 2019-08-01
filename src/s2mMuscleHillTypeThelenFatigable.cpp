#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillTypeThelenFatigable.h"

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mString &s,
        const s2mString &dynamicFatigueType) :
    s2mMuscleHillTypeThelen(s),
    s2mMuscleFatigable (dynamicFatigueType)
{
    setType();
}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mMuscleGeometry &g,
        const s2mMuscleCaracteristics &c,
        const s2mMusclePathChangers &w,
        const s2mMuscleStateActual &s,
        const s2mString &dynamicFatigueType) :
    s2mMuscleHillTypeThelen(g, c, w, s),
    s2mMuscleFatigable (dynamicFatigueType)
{
    setType();
}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mString &n,
        const s2mMuscleGeometry &g,
        const s2mMuscleCaracteristics &c,
        const s2mMusclePathChangers &w,
        const s2mMuscleStateActual &s,
        const s2mString &dynamicFatigueType) :
    s2mMuscleHillTypeThelen(n, g, c, w, s),
    s2mMuscleFatigable (dynamicFatigueType)
{
    setType();
}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mMuscle &m) :
    s2mMuscleHillTypeThelen(m),
    s2mMuscleFatigable (m)
{
    setType();
}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const std::shared_ptr<s2mMuscle> m) :
    s2mMuscleHillTypeThelen(m),
    s2mMuscleFatigable(m)
{
    setType();
}

void s2mMuscleHillTypeThelenFatigable::computeFlCE(const s2mMuscleStateActual &EMG)
{
    s2mMuscleHillTypeThelen::computeFlCE(EMG);   
    // Do something with m_FlCE and m_caract.fatigueParameters
    m_FlCE *= m_fatigueState->activeFibers();
}

void s2mMuscleHillTypeThelenFatigable::setType()
{
    m_type = "HillThelenFatigable";
}
