#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillThelenTypeFatigable.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/FatigueState.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable() :
    internalforce::muscles::HillThelenType(),
    internalforce::muscles::FatigueModel (
        internalforce::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillThelenType(name, geometry, characteristics),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillThelenType(name, geometry, characteristics, emg),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const PathModifiers &pathModifiers,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillThelenType(name, geometry, characteristics, pathModifiers),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const PathModifiers& pathModifiers,
    const internalforce::muscles::State& emg,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillThelenType(name, geometry, characteristics, pathModifiers,
                                    emg),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const internalforce::muscles::Muscle &other) :
    internalforce::muscles::HillThelenType (other),
    internalforce::muscles::FatigueModel (
        dynamic_cast<const internalforce::muscles::FatigueModel&>(other))
{

}

internalforce::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const std::shared_ptr<internalforce::muscles::Muscle> other) :
    internalforce::muscles::HillThelenType (other),
    internalforce::muscles::FatigueModel (
        std::dynamic_pointer_cast<internalforce::muscles::FatigueModel>(other))
{

}

internalforce::muscles::HillThelenTypeFatigable
internalforce::muscles::HillThelenTypeFatigable::DeepCopy() const
{
    internalforce::muscles::HillThelenTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::HillThelenTypeFatigable::DeepCopy(
    const internalforce::muscles::HillThelenTypeFatigable &other)
{
    internalforce::muscles::HillThelenType::DeepCopy(other);
    internalforce::muscles::FatigueModel::DeepCopy(other);
}

void internalforce::muscles::HillThelenTypeFatigable::computeFlCE(
    const internalforce::muscles::State &emg)
{
    internalforce::muscles::HillThelenType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void internalforce::muscles::HillThelenTypeFatigable::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
}
