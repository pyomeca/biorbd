#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillDeGrooteTypeFatigable.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/FatigueState.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable() :
    internal_forces::muscles::HillDeGrooteType(),
    internal_forces::muscles::FatigueModel (
        internal_forces::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internal_forces::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillDeGrooteType(name, geometry, characteristics),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internal_forces::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &emg,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillDeGrooteType(name, geometry, characteristics, emg),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internal_forces::Geometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String& name,
    const internal_forces::Geometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers& pathModifiers,
    const internal_forces::muscles::State& emg,
    internal_forces::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internal_forces::muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers,
                                    emg),
    internal_forces::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::HillDeGrooteType (other),
    internal_forces::muscles::FatigueModel (
        dynamic_cast<const internal_forces::muscles::FatigueModel&>(other))
{

}

internal_forces::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::HillDeGrooteType (other),
    internal_forces::muscles::FatigueModel (
        std::dynamic_pointer_cast<internal_forces::muscles::FatigueModel>(other))
{

}

internal_forces::muscles::HillDeGrooteTypeFatigable
internal_forces::muscles::HillDeGrooteTypeFatigable::DeepCopy() const
{
    internal_forces::muscles::HillDeGrooteTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::HillDeGrooteTypeFatigable::DeepCopy(
    const internal_forces::muscles::HillDeGrooteTypeFatigable &other)
{
    internal_forces::muscles::HillDeGrooteType::DeepCopy(other);
    internal_forces::muscles::FatigueModel::DeepCopy(other);
}

void internal_forces::muscles::HillDeGrooteTypeFatigable::computeFlCE(
    const internal_forces::muscles::State &emg)
{
    internal_forces::muscles::HillDeGrooteType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void internal_forces::muscles::HillDeGrooteTypeFatigable::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE;
}
