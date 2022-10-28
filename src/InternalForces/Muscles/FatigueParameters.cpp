#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/FatigueParameters.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::FatigueParameters::FatigueParameters(
    const utils::Scalar& _fatigueRate,
    const utils::Scalar& _recoveryRate,
    const utils::Scalar& _developFactor,
    const utils::Scalar& recoveryFactor):
    m_fatigueRate(std::make_shared<utils::Scalar>(_fatigueRate)),
    m_recoveryRate(std::make_shared<utils::Scalar>(_recoveryRate)),
    m_developFactor(std::make_shared<utils::Scalar>(_developFactor)),
    m_recoveryFactor(std::make_shared<utils::Scalar>(recoveryFactor))
{

}

internal_forces::muscles::FatigueParameters
internal_forces::muscles::FatigueParameters::DeepCopy() const
{
    internal_forces::muscles::FatigueParameters copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::FatigueParameters::DeepCopy(const
        internal_forces::muscles::FatigueParameters &other)
{
    *m_fatigueRate = *other.m_fatigueRate;
    *m_recoveryRate = *other.m_recoveryRate;
    *m_developFactor = *other.m_developFactor;
    *m_recoveryFactor = *other.m_recoveryFactor;
}

void internal_forces::muscles::FatigueParameters::setFatigueRate(
    const utils::Scalar& fatigueRate)
{
    *m_fatigueRate = fatigueRate;
}
const utils::Scalar& internal_forces::muscles::FatigueParameters::fatigueRate()
const
{
    return *m_fatigueRate;
}

void internal_forces::muscles::FatigueParameters::setRecoveryRate(
    const utils::Scalar& recoveryRate)
{
    *m_recoveryRate = recoveryRate;
}
const utils::Scalar& internal_forces::muscles::FatigueParameters::recoveryRate()
const
{
    return *m_recoveryRate;
}

void internal_forces::muscles::FatigueParameters::setDevelopFactor(
    const utils::Scalar& developFactor)
{
    *m_developFactor = developFactor;
}
const utils::Scalar& internal_forces::muscles::FatigueParameters::developFactor()
const
{
    return *m_developFactor;
}

void internal_forces::muscles::FatigueParameters::setRecoveryFactor(
    const utils::Scalar& recoveryFactor)
{
    *m_recoveryFactor = recoveryFactor;
}
const utils::Scalar&
internal_forces::muscles::FatigueParameters::recoveryFactor() const
{
    return *m_recoveryFactor;
}
