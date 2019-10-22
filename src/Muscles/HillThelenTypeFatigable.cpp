#define BIORBD_API_EXPORTS
#include "Muscles/HillThelenTypeFatigable.h"

#include "Utils/String.h"
#include "Muscles/FatigueState.h"

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable() :
    biorbd::muscles::HillThelenType(),
    biorbd::muscles::Fatigable (biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        const biorbd::muscles::StateDynamics &dynamicState,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, dynamicState),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Characteristics &characteristics,
        const biorbd::muscles::PathChangers &pathChangers,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, pathChangers),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathChangers& pathChangers,
        const biorbd::muscles::StateDynamics& dynamicState,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, pathChangers, dynamicState),
    biorbd::muscles::Fatigable (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const biorbd::muscles::Muscle &muscle) :
    biorbd::muscles::HillThelenType (muscle),
    biorbd::muscles::Fatigable (dynamic_cast<const biorbd::muscles::Fatigable&>(muscle))
{

}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
        const std::shared_ptr<biorbd::muscles::Muscle> muscle) :
    biorbd::muscles::HillThelenType (muscle),
    biorbd::muscles::Fatigable (std::dynamic_pointer_cast<biorbd::muscles::Fatigable>(muscle))
{

}

biorbd::muscles::HillThelenTypeFatigable biorbd::muscles::HillThelenTypeFatigable::DeepCopy() const
{
    biorbd::muscles::HillThelenTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::HillThelenTypeFatigable::DeepCopy(const biorbd::muscles::HillThelenTypeFatigable &other)
{
    biorbd::muscles::HillThelenType::DeepCopy(other);
    biorbd::muscles::Fatigable::DeepCopy(other);
}

void biorbd::muscles::HillThelenTypeFatigable::computeFlCE(const biorbd::muscles::StateDynamics &EMG)
{
    biorbd::muscles::HillThelenType::computeFlCE(EMG);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void biorbd::muscles::HillThelenTypeFatigable::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
}
