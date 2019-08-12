#ifndef S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_XIA_H
#define S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_XIA_H

#include "biorbdConfig.h"
#include "s2mMuscleFatigueDynamicState.h"

class BIORBD_API s2mMuscleFatigueDynamicStateXia : public s2mMuscleFatigueDynamicState
{
public:
    s2mMuscleFatigueDynamicStateXia(
            double active = 0,
            double fatigued = 0,
            double resting = 1);

    s2mMuscleFatigueDynamicStateXia(const std::shared_ptr<s2mMuscleFatigueState> m);

    virtual void timeDerivativeState(
            const s2mMuscleStateDynamics &emg,
            const s2mMuscleCaracteristics &caract
     );
protected:
    virtual void setType();

};

#endif // S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_XIA_H
