#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenTypeFatigable.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/FatigueState.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable() :
    internal_forces::muscles::HillThelenType(),
    internal_forces::muscles::FatigueModel (
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internal_forces::muscles::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillThelenType(name, geometry, characteristics),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internal_forces::muscles::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillThelenType(name, geometry, characteristics, emg),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internal_forces::muscles::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillThelenType(name, geometry, characteristics, pathModifiers),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String& name,
    const internal_forces::muscles::Geometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers& pathModifiers,
    const internal_forces::muscles::State& emg,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillThelenType(name, geometry, characteristics, pathModifiers,
                                    emg),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::HillThelenType (other),
    internal_forces::muscles::FatigueModel (
        dynamic_cast<const internal_forces::muscles::FatigueModel&>(other))
{

}

internal_forces::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::HillThelenType (other),
    internal_forces::muscles::FatigueModel (
        std::dynamic_pointer_cast<internal_forces::muscles::FatigueModel>(other))
{

}

internal_forces::muscles::HillThelenTypeFatigable
internal_forces::muscles::HillThelenTypeFatigable::DeepCopy() const
{
    internal_forces::muscles::HillThelenTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::HillThelenTypeFatigable::DeepCopy(
    const internal_forces::muscles::HillThelenTypeFatigable &other)
{
    internal_forces::muscles::HillThelenType::DeepCopy(other);
    internal_forces::muscles::FatigueModel::DeepCopy(other);
}

void internal_forces::muscles::HillThelenTypeFatigable::computeFlCE(
    const internal_forces::muscles::State &emg)
{
    internal_forces::muscles::HillThelenType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void internal_forces::muscles::HillThelenTypeFatigable::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
}
