#ifndef BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H

#include "biorbdConfig.h"

#include "Muscles/HillTypeThelen.h"
#include "Muscles/Fatigable.h"
#include "Muscles/PathChangers.h"
#include "Muscles/StateDynamics.h"

namespace biorbd { namespace muscles {

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
    HillTypeThelenFatigable(
            const biorbd::utils::String& s= "",
            const biorbd::utils::String& dynamicFatigueType = "Simple");
    HillTypeThelenFatigable(const Geometry& g,
            const Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics(),
            const biorbd::utils::String& dynamicFatigueType = "Simple");
    HillTypeThelenFatigable(
            const biorbd::utils::String& n,
            const Geometry& g,
            const Caracteristics& c,
            const biorbd::muscles::PathChangers & w= biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & s= biorbd::muscles::StateDynamics(),
            const biorbd::utils::String& dynamicFatigueType = "Simple");
    HillTypeThelenFatigable(const biorbd::muscles::Muscle& m);
    HillTypeThelenFatigable(const std::shared_ptr<biorbd::muscles::Muscle> m);
    virtual ~HillTypeThelenFatigable(){}

    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG);

protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_THELEN_FATIGABLE_H
