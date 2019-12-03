#define BIORBD_API_EXPORTS
#include "Muscles/FatigueParameters.h"

biorbd::muscles::FatigueParameters::FatigueParameters(
        double _fatigueRate,
        double _recoveryRate,
        double _developFactor,
        double recoveryFactor):
    m_fatigueRate(std::make_shared<double>(_fatigueRate)),
    m_recoveryRate(std::make_shared<double>(_recoveryRate)),
    m_developFactor(std::make_shared<double>(_developFactor)),
    m_recoveryFactor(std::make_shared<double>(recoveryFactor))
{

}

biorbd::muscles::FatigueParameters biorbd::muscles::FatigueParameters::DeepCopy() const
{
    biorbd::muscles::FatigueParameters copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::FatigueParameters::DeepCopy(const biorbd::muscles::FatigueParameters &other)
{
    *m_fatigueRate = *other.m_fatigueRate;
    *m_recoveryRate = *other.m_recoveryRate;
    *m_developFactor = *other.m_developFactor;
    *m_recoveryFactor = *other.m_recoveryFactor;
}

void biorbd::muscles::FatigueParameters::setFatigueRate(double fatigueRate)
{
    *m_fatigueRate = fatigueRate;
}
double biorbd::muscles::FatigueParameters::fatigueRate() const
{
    return *m_fatigueRate;
}

void biorbd::muscles::FatigueParameters::setRecoveryRate(double recoveryRate)
{
    *m_recoveryRate = recoveryRate;
}
double biorbd::muscles::FatigueParameters::recoveryRate() const
{
    return *m_recoveryRate;
}

void biorbd::muscles::FatigueParameters::setDevelopFactor(double developFactor)
{
    *m_developFactor = developFactor;
}
double biorbd::muscles::FatigueParameters::developFactor() const
{
    return *m_developFactor;
}

void biorbd::muscles::FatigueParameters::setRecoveryFactor(double recoveryFactor)
{
    *m_recoveryFactor = recoveryFactor;
}
double biorbd::muscles::FatigueParameters::recoveryFactor() const
{
    return *m_recoveryFactor;
}
