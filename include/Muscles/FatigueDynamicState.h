#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H

#include "biorbdConfig.h"
#include "Muscles/FatigueState.h"

namespace biorbd { namespace muscles {

class StateDynamics;
class Caracteristics;
class BIORBD_API FatigueDynamicState : public biorbd::muscles::FatigueState
{
public:
    FatigueDynamicState(
            double active = 0,
            double fatigued = 0,
            double resting = 1);

    FatigueDynamicState(const std::shared_ptr<biorbd::muscles::FatigueState> m);

    double activeFibersDot() const;
    double fatiguedFibersDot() const;
    double restingFibersDot() const;

    virtual void timeDerivativeState(
            const biorbd::muscles::StateDynamics &emg,
            const biorbd::muscles::Caracteristics &caract
     ) = 0;

protected:
    double m_activeFibersDot;
    double m_fatiguedFibersDot;
    double m_restingFibersDot;

    virtual void setType();
};

}}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
