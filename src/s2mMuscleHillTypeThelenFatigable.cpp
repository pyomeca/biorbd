#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillTypeThelenFatigable.h"

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mString &s) :
    s2mMuscleHillTypeThelen(s),
    m_fatigueRate(0.01),
    m_recoveryRate(0.002),
    m_developFactor(10),
    m_recoverFactor(10)
{
    setType();
}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(
        const s2mMuscleGeometry &g,
        const s2mMuscleCaracteristics &c,
        const s2mMusclePathChangers &w,
        const s2mMuscleStateActual &s) :
    s2mMuscleHillTypeThelen(g, c, w, s),
    m_fatigueRate(c.fatigueParameters().fatigueRate()),
    m_recoveryRate(c.fatigueParameters().recoveryRate()),
    m_developFactor(c.fatigueParameters().developFactor()),
    m_recoverFactor(c.fatigueParameters().recoverFactor())
{

}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(
        const s2mString &n,
        const s2mMuscleGeometry &g,
        const s2mMuscleCaracteristics &c,
        const s2mMusclePathChangers &w,
        const s2mMuscleStateActual &s) :
    s2mMuscleHillTypeThelen(n, g, c, w, s),
    m_fatigueRate(c.fatigueParameters().fatigueRate()),
    m_recoveryRate(c.fatigueParameters().recoveryRate()),
    m_developFactor(c.fatigueParameters().developFactor()),
    m_recoverFactor(c.fatigueParameters().recoverFactor())
{

}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const s2mMuscle &m) :
    s2mMuscleHillTypeThelen(m),
    m_fatigueRate(0.01),
    m_recoveryRate(0.002),
    m_developFactor(10),
    m_recoverFactor(10)
{

}

s2mMuscleHillTypeThelenFatigable::s2mMuscleHillTypeThelenFatigable(const std::shared_ptr<s2mMuscle> m) :
    s2mMuscleHillTypeThelen(m),
    m_fatigueRate(0.01),
    m_recoveryRate(0.002),
    m_developFactor(10),
    m_recoverFactor(10)
{

}

void s2mMuscleHillTypeThelenFatigable::computeFlCE(const s2mMuscleStateActual &EMG)
{
    s2mMuscleHillTypeThelen::computeFlCE(EMG);   
    // Do something with m_FlCE and m_caract.fatigueParameters
    //m_FlCE *= fatigueState
}

void s2mMuscleHillTypeThelenFatigable::timeDerivatedFatigueState(const s2mMuscleCaracteristics &c, const s2mMuscleStateActual &EMG)
{
    c.fatigueParameters().fatigueRate();

}

void s2mMuscleHillTypeThelenFatigable::setType()
{
    m_type = "HillThelenFatigable";
}
