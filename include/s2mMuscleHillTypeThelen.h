#ifndef S2M_MUSCLE_HILL_TYPE_THELEN_H
#define S2M_MUSCLE_HILL_TYPE_THELEN_H

#include "biorbdConfig.h"
#include "s2mMuscleHillType.h"

class BIORBD_API s2mMuscleHillTypeThelen : public s2mMuscleHillType
{
public:
    s2mMuscleHillTypeThelen(const biorbd::utils::String& s= "") : s2mMuscleHillType(s){setType();}
    s2mMuscleHillTypeThelen(
            const s2mMuscleGeometry& g,
            const s2mMuscleCaracteristics& c,
            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics());

    s2mMuscleHillTypeThelen(
            const biorbd::utils::String& n,
            const s2mMuscleGeometry& g,
            const s2mMuscleCaracteristics& c,
            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics());
    s2mMuscleHillTypeThelen(const s2mMuscle& m);
    s2mMuscleHillTypeThelen(const std::shared_ptr<s2mMuscle> m);
    virtual ~s2mMuscleHillTypeThelen();

    virtual void computeFlPE();
    virtual void computeFlCE(const s2mMuscleStateDynamics &emg);

protected:
    virtual void setType();
};

#endif // S2M_MUSCLE_HILL_TYPE_THELEN_H
