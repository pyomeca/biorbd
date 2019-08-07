#ifndef S2M_MUSCLE_HILL_TYPE_THELEN_FATIGABLE_H
#define S2M_MUSCLE_HILL_TYPE_THELEN_FATIGABLE_H

#include "biorbdConfig.h"

#include "s2mMuscleHillTypeThelen.h"
#include "s2mMuscleFatigable.h"
#include "s2mMusclePathChangers.h"
#include "s2mMuscleStateDynamics.h"

///
/// \brief The s2mMuscleHillTypeThelenFatigable class
/// Class of a Thelen fatigable type.
/// Note that useful defaults values for the s2mMuscleFatigueParam caracteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API s2mMuscleHillTypeThelenFatigable : public s2mMuscleHillTypeThelen, public s2mMuscleFatigable
{
public:
    s2mMuscleHillTypeThelenFatigable(
            const s2mString& s= "",
            const s2mString& dynamicFatigueType = "Simple");
    s2mMuscleHillTypeThelenFatigable(const s2mMuscleGeometry& g,
            const s2mMuscleCaracteristics& c,
            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics(),
            const s2mString& dynamicFatigueType = "Simple");
    s2mMuscleHillTypeThelenFatigable(
            const s2mString& n,
            const s2mMuscleGeometry& g,
            const s2mMuscleCaracteristics& c,
            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics(),
            const s2mString& dynamicFatigueType = "Simple");
    s2mMuscleHillTypeThelenFatigable(const s2mMuscle& m);
    s2mMuscleHillTypeThelenFatigable(const std::shared_ptr<s2mMuscle> m);
    virtual ~s2mMuscleHillTypeThelenFatigable(){}

    virtual void computeFlCE(const s2mMuscleStateDynamics &EMG);

protected:
    virtual void setType();

};

#endif // S2M_MUSCLE_HILL_TYPE_THELEN_FATIGABLE_H
