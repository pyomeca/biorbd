#define BIORBD_API_EXPORTS
#include "InternalForces/Ligaments/LigamentCharacteristics.h"

using namespace BIORBD_NAMESPACE;

internal_forces::ligaments::LigamentCharacteristics::LigamentCharacteristics() :
    m_ligamentSlackLength(std::make_shared<utils::Scalar>(0)),
    m_cste_damping(std::make_shared<utils::Scalar>(0)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(1))
{

}

internal_forces::ligaments::LigamentCharacteristics::LigamentCharacteristics(
    const internal_forces::ligaments::LigamentCharacteristics &other) :
    m_ligamentSlackLength(other.m_ligamentSlackLength),
    m_cste_damping(other.m_cste_damping),
    m_cste_maxShorteningSpeed(other.m_cste_maxShorteningSpeed)
{

}

internal_forces::ligaments::LigamentCharacteristics::LigamentCharacteristics(
    const utils::Scalar& ligamentSlackLength,
    const utils::Scalar& cste_damping,
    const utils::Scalar& cste_maxShorteningSpeed):
    m_ligamentSlackLength(std::make_shared<utils::Scalar>(ligamentSlackLength)),
    m_cste_damping(std::make_shared<utils::Scalar>(cste_damping)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(cste_maxShorteningSpeed))

{

}

internal_forces::ligaments::LigamentCharacteristics::~LigamentCharacteristics()
{

}

internal_forces::ligaments::LigamentCharacteristics internal_forces::ligaments::LigamentCharacteristics::DeepCopy()
const
{
    internal_forces::ligaments::LigamentCharacteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ligaments::LigamentCharacteristics::DeepCopy(
    const internal_forces::ligaments::LigamentCharacteristics &other)
{
    *m_ligamentSlackLength = *other.m_ligamentSlackLength;
    *m_cste_damping = *other.m_cste_damping;
    *m_cste_maxShorteningSpeed = *other.m_cste_maxShorteningSpeed;
}

void internal_forces::ligaments::LigamentCharacteristics::setLigamentSlackLength(
    const utils::Scalar& val)
{
    *m_ligamentSlackLength = val;
}
const utils::Scalar& internal_forces::ligaments::LigamentCharacteristics::ligamentSlackLength() const
{
    return *m_ligamentSlackLength;
}

void internal_forces::ligaments::LigamentCharacteristics::setMaxShorteningSpeed(
    const utils::Scalar& val)
{
    *m_cste_maxShorteningSpeed= val;
}
const utils::Scalar& internal_forces::ligaments::LigamentCharacteristics::maxShorteningSpeed() const
{
    return *m_cste_maxShorteningSpeed;
}

void internal_forces::ligaments::LigamentCharacteristics::setDampingParam(
    const utils::Scalar& val)
{
    *m_cste_damping = val;
}
const utils::Scalar& internal_forces::ligaments::LigamentCharacteristics::dampingParam() const
{
    return *m_cste_damping;
}
