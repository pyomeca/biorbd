#ifndef S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_H
#define S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_H

#include "biorbdConfig.h"
#include "Muscles/FatigueState.h"

class s2mMuscleStateDynamics;
class s2mMuscleCaracteristics;
class BIORBD_API s2mMuscleFatigueDynamicState : public s2mMuscleFatigueState
{
public:
    s2mMuscleFatigueDynamicState(
            double active = 0,
            double fatigued = 0,
            double resting = 1);

    s2mMuscleFatigueDynamicState(const std::shared_ptr<s2mMuscleFatigueState> m);

    double activeFibersDot() const;
    double fatiguedFibersDot() const;
    double restingFibersDot() const;

    virtual void timeDerivativeState(
            const s2mMuscleStateDynamics &emg,
            const s2mMuscleCaracteristics &caract
     ) = 0;

protected:
    double m_activeFibersDot;
    double m_fatiguedFibersDot;
    double m_restingFibersDot;

    virtual void setType();
};

#endif // S2M_MUSCLE_FATIGUE_DYNAMIC_STATE_H
