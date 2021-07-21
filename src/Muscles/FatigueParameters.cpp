#define BIORBD_API_EXPORTS
#include "Muscles/FatigueParameters.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::FatigueParameters::FatigueParameters(
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

muscles::FatigueParameters
muscles::FatigueParameters::DeepCopy() const
{
    muscles::FatigueParameters copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::FatigueParameters::DeepCopy(const
        muscles::FatigueParameters &other)
{
    *m_fatigueRate = *other.m_fatigueRate;
    *m_recoveryRate = *other.m_recoveryRate;
    *m_developFactor = *other.m_developFactor;
    *m_recoveryFactor = *other.m_recoveryFactor;
}

void muscles::FatigueParameters::setFatigueRate(
    const utils::Scalar& fatigueRate)
{
    *m_fatigueRate = fatigueRate;
}
const utils::Scalar& muscles::FatigueParameters::fatigueRate()
const
{
    return *m_fatigueRate;
}

void muscles::FatigueParameters::setRecoveryRate(
    const utils::Scalar& recoveryRate)
{
    *m_recoveryRate = recoveryRate;
}
const utils::Scalar& muscles::FatigueParameters::recoveryRate()
const
{
    return *m_recoveryRate;
}

void muscles::FatigueParameters::setDevelopFactor(
    const utils::Scalar& developFactor)
{
    *m_developFactor = developFactor;
}
const utils::Scalar& muscles::FatigueParameters::developFactor()
const
{
    return *m_developFactor;
}

void muscles::FatigueParameters::setRecoveryFactor(
    const utils::Scalar& recoveryFactor)
{
    *m_recoveryFactor = recoveryFactor;
}
const utils::Scalar&
muscles::FatigueParameters::recoveryFactor() const
{
    return *m_recoveryFactor;
}
