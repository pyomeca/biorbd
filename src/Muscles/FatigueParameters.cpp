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
    *copy.m_fatigueRate = *m_fatigueRate;
    *copy.m_recoveryRate = *m_recoveryRate;
    *copy.m_developFactor = *m_developFactor;
    *copy.m_recoveryFactor = *m_recoveryFactor;
    return copy;
}

void biorbd::muscles::FatigueParameters::DeepCopy(const biorbd::muscles::FatigueParameters &other)
{
    *m_fatigueRate = *other.m_fatigueRate;
    *m_recoveryRate = *other.m_recoveryRate;
    *m_developFactor = *other.m_developFactor;
    *m_recoveryFactor = *other.m_recoveryFactor;
}

double biorbd::muscles::FatigueParameters::fatigueRate() const
{
    return *m_fatigueRate;
}

double biorbd::muscles::FatigueParameters::recoveryRate() const
{
    return *m_recoveryRate;
}

double biorbd::muscles::FatigueParameters::developFactor() const
{
    return *m_developFactor;
}

double biorbd::muscles::FatigueParameters::recoveryFactor() const
{
    return *m_recoveryFactor;
}

void biorbd::muscles::FatigueParameters::fatigueRate(double fatigueRate)
{
    *m_fatigueRate = fatigueRate;
}

void biorbd::muscles::FatigueParameters::recoveryRate(double recoveryRate)
{
    *m_recoveryRate = recoveryRate;
}

void biorbd::muscles::FatigueParameters::developFactor(double developFactor)
{
    *m_developFactor = developFactor;
}

void biorbd::muscles::FatigueParameters::recoveryFactor(double recoveryFactor)
{
    *m_recoveryFactor = recoveryFactor;
}




