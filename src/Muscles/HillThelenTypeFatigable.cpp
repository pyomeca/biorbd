#define BIORBD_API_EXPORTS
#include "Muscles/HillThelenTypeFatigable.h"

#include "Utils/String.h"
#include "Muscles/FatigueState.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable() :
    muscles::HillThelenType(),
    muscles::FatigueModel (
        muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillThelenType(name, geometry, characteristics),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillThelenType(name, geometry, characteristics, emg),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillThelenType(name, geometry, characteristics, pathModifiers),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers& pathModifiers,
    const muscles::State& emg,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillThelenType(name, geometry, characteristics, pathModifiers,
                                    emg),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const muscles::Muscle &other) :
    muscles::HillThelenType (other),
    muscles::FatigueModel (
        dynamic_cast<const muscles::FatigueModel&>(other))
{

}

muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::HillThelenType (other),
    muscles::FatigueModel (
        std::dynamic_pointer_cast<muscles::FatigueModel>(other))
{

}

muscles::HillThelenTypeFatigable
muscles::HillThelenTypeFatigable::DeepCopy() const
{
    muscles::HillThelenTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillThelenTypeFatigable::DeepCopy(
    const muscles::HillThelenTypeFatigable &other)
{
    muscles::HillThelenType::DeepCopy(other);
    muscles::FatigueModel::DeepCopy(other);
}

void muscles::HillThelenTypeFatigable::computeFlCE(
    const muscles::State &emg)
{
    muscles::HillThelenType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void muscles::HillThelenTypeFatigable::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
}
