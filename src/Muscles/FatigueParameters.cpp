#define BIORBD_API_EXPORTS
#include "Muscles/FatigueParameters.h"

biorbd::muscles::FatigueParameters::FatigueParameters(
        double _fatigueRate,
        double _recoveryRate,
        double _developFactor,
        double recoveryFactor):
    m_fatigueRate(_fatigueRate),
    m_recoveryRate(_recoveryRate),
    m_developFactor(_developFactor),
    m_recoveryFactor(recoveryFactor)
{

}

double biorbd::muscles::FatigueParameters::fatigueRate() const
{
    return m_fatigueRate;
}

double biorbd::muscles::FatigueParameters::recoveryRate() const
{
    return m_recoveryRate;
}

double biorbd::muscles::FatigueParameters::developFactor() const
{
    return m_developFactor;
}

double biorbd::muscles::FatigueParameters::recoveryFactor() const
{
    return m_recoveryFactor;
}

void biorbd::muscles::FatigueParameters::fatigueRate(double fatigueRate)
{
    m_fatigueRate = fatigueRate;
}

void biorbd::muscles::FatigueParameters::recoveryRate(double recoveryRate)
{
    m_recoveryRate = recoveryRate;
}

void biorbd::muscles::FatigueParameters::developFactor(double developFactor)
{
    m_developFactor = developFactor;
}

void biorbd::muscles::FatigueParameters::recoveryFactor(double recoveryFactor)
{
    m_recoveryFactor = recoveryFactor;
}




