#define BIORBD_API_EXPORTS
#include "s2mMuscleFatigueParam.h"

s2mMuscleFatigueParam::s2mMuscleFatigueParam(double _fatigueRate, double _recoveryRate, double _developFactor, double recoveryFactor):
    m_fatigueRate(_fatigueRate),
    m_recoveryRate(_recoveryRate),
    m_developFactor(_developFactor),
    m_recoveryFactor(recoveryFactor)
{

}

double s2mMuscleFatigueParam::fatigueRate() const
{
    return m_fatigueRate;
}

double s2mMuscleFatigueParam::recoveryRate() const
{
    return m_recoveryRate;
}

double s2mMuscleFatigueParam::developFactor() const
{
    return m_developFactor;
}

double s2mMuscleFatigueParam::recoveryFactor() const
{
    return m_recoveryFactor;
}

void s2mMuscleFatigueParam::fatigueRate(double fatigueRate)
{
    m_fatigueRate = fatigueRate;
}

void s2mMuscleFatigueParam::recoveryRate(double recoveryRate)
{
    m_recoveryRate = recoveryRate;
}

void s2mMuscleFatigueParam::developFactor(double developFactor)
{
    m_developFactor = developFactor;
}

void s2mMuscleFatigueParam::recoveryFactor(double recoveryFactor)
{
    m_recoveryFactor = recoveryFactor;
}




