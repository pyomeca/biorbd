#define BIORBD_API_EXPORTS
#include "Muscles/HillDeGrooteTypeFatigable.h"

#include "Utils/String.h"
#include "Muscles/FatigueState.h"

using namespace BIORBD_NAMESPACE;

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable() :
    muscles::HillDeGrooteType(),
    muscles::FatigueModel (
        muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillDeGrooteType(name, geometry, characteristics),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State &emg,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillDeGrooteType(name, geometry, characteristics, emg),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers& pathModifiers,
    const muscles::State& emg,
    muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers,
                                    emg),
    muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const muscles::Muscle &other) :
    muscles::HillDeGrooteType (other),
    muscles::FatigueModel (
        dynamic_cast<const muscles::FatigueModel&>(other))
{

}

muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::HillDeGrooteType (other),
    muscles::FatigueModel (
        std::dynamic_pointer_cast<muscles::FatigueModel>(other))
{

}

muscles::HillDeGrooteTypeFatigable
muscles::HillDeGrooteTypeFatigable::DeepCopy() const
{
    muscles::HillDeGrooteTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillDeGrooteTypeFatigable::DeepCopy(
    const muscles::HillDeGrooteTypeFatigable &other)
{
    muscles::HillDeGrooteType::DeepCopy(other);
    muscles::FatigueModel::DeepCopy(other);
}

void muscles::HillDeGrooteTypeFatigable::computeFlCE(
    const muscles::State &emg)
{
    muscles::HillDeGrooteType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void muscles::HillDeGrooteTypeFatigable::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE;
}
