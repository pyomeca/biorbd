#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/HillDeGrooteTypeFatigable.h"

#include "Utils/String.h"
#include "InternalForces/Muscles/FatigueState.h"

using namespace BIORBD_NAMESPACE;

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable() :
    internalforce::muscles::HillDeGrooteType(),
    internalforce::muscles::FatigueModel (
        internalforce::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillDeGrooteType(name, geometry, characteristics),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &emg,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillDeGrooteType(name, geometry, characteristics, emg),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String &name,
    const internalforce::muscles::Geometry &geometry,
    const internalforce::muscles::Characteristics &characteristics,
    const PathModifiers &pathModifiers,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const utils::String& name,
    const internalforce::muscles::Geometry& geometry,
    const internalforce::muscles::Characteristics& characteristics,
    const PathModifiers& pathModifiers,
    const internalforce::muscles::State& emg,
    internalforce::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    internalforce::muscles::HillDeGrooteType(name, geometry, characteristics, pathModifiers,
                                    emg),
    internalforce::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const internalforce::muscles::Muscle &other) :
    internalforce::muscles::HillDeGrooteType (other),
    internalforce::muscles::FatigueModel (
        dynamic_cast<const internalforce::muscles::FatigueModel&>(other))
{

}

internalforce::muscles::HillDeGrooteTypeFatigable::HillDeGrooteTypeFatigable(
    const std::shared_ptr<internalforce::muscles::Muscle> other) :
    internalforce::muscles::HillDeGrooteType (other),
    internalforce::muscles::FatigueModel (
        std::dynamic_pointer_cast<internalforce::muscles::FatigueModel>(other))
{

}

internalforce::muscles::HillDeGrooteTypeFatigable
internalforce::muscles::HillDeGrooteTypeFatigable::DeepCopy() const
{
    internalforce::muscles::HillDeGrooteTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::muscles::HillDeGrooteTypeFatigable::DeepCopy(
    const internalforce::muscles::HillDeGrooteTypeFatigable &other)
{
    internalforce::muscles::HillDeGrooteType::DeepCopy(other);
    internalforce::muscles::FatigueModel::DeepCopy(other);
}

void internalforce::muscles::HillDeGrooteTypeFatigable::computeFlCE(
    const internalforce::muscles::State &emg)
{
    internalforce::muscles::HillDeGrooteType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void internalforce::muscles::HillDeGrooteTypeFatigable::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::HILL_DE_GROOTE_FATIGABLE;
}
