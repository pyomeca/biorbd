#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleFatigueParam.h"

s2mMuscleFatigueParam::s2mMuscleFatigueParam(double fatigueRate, double recoveryRate, double developFactor, double recoverFactor)
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

double s2mMuscleFatigueParam::recoverFactor() const
{
    return m_recoverFactor;
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

void s2mMuscleFatigueParam::recoverFactor(double recoverFactor)
{
    m_recoverFactor = recoverFactor;
}




