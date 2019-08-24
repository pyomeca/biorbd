#ifndef BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H

#include "biorbdConfig.h"

#include "Muscles/HillTypeThelen.h"
#include "Muscles/Fatigable.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {

///
/// \brief The HillTypeThelenFatigable class
/// Class of a Thelen fatigable type.
/// Note that useful defaults values for the FatigueParameters caracteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API HillTypeThelenFatigable : public biorbd::muscles::HillTypeThelen, public biorbd::muscles::Fatigable
{
public:
    HillTypeThelenFatigable();
    HillTypeThelenFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract);
    HillTypeThelenFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::utils::String& dynamicFatigueType);
    HillTypeThelenFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState
            );
    HillTypeThelenFatigable(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Caracteristics& caract,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState,
            const biorbd::utils::String& dynamicFatigueType);
    HillTypeThelenFatigable(
            const biorbd::muscles::Muscle& muscle);
    HillTypeThelenFatigable(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);

    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H
