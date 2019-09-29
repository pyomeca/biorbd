#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H

#include "biorbdConfig.h"
#include "Muscles/FatigueState.h"

namespace biorbd {
namespace muscles {
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
    void DeepCopy(const biorbd::muscles::FatigueDynamicState& other);

    double activeFibersDot() const;
    double fatiguedFibersDot() const;
    double restingFibersDot() const;

    virtual void timeDerivativeState(
            const biorbd::muscles::StateDynamics &emg,
            const biorbd::muscles::Caracteristics &caract
     ) = 0;

protected:
    std::shared_ptr<double> m_activeFibersDot;
    std::shared_ptr<double> m_fatiguedFibersDot;
    std::shared_ptr<double> m_restingFibersDot;

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_H
