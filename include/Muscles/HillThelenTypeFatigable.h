#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H

#include "biorbdConfig.h"

#include "Muscles/HillThelenType.h"
#include "Muscles/Fatigable.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {

///
/// \brief The HillThelenTypeFatigable class
/// Class of a Thelen fatigable type.
/// Note that useful defaults values for the FatigueParameters caracteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API HillThelenTypeFatigable : public biorbd::muscles::HillThelenType, public biorbd::muscles::Fatigable
{
public:
    HillThelenTypeFatigable();
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::StateDynamics& dynamicState,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);
    HillThelenTypeFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState,
            biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);
    HillThelenTypeFatigable(
            const biorbd::muscles::Muscle& muscle);
    HillThelenTypeFatigable(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    biorbd::muscles::HillThelenTypeFatigable DeepCopy() const;
    void DeepCopy(const biorbd::muscles::HillThelenTypeFatigable& other);

    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
