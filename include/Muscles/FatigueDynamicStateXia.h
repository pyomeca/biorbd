#ifndef BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
#define BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H

#include "biorbdConfig.h"
#include "Muscles/FatigueDynamicState.h"

namespace biorbd {
namespace muscles {

class BIORBD_API FatigueDynamicStateXia : public biorbd::muscles::FatigueDynamicState
{
public:
    FatigueDynamicStateXia(
            double active = 1,
            double fatigued = 0,
            double resting = 0);
    FatigueDynamicStateXia(const std::shared_ptr<biorbd::muscles::FatigueState> m);
    biorbd::muscles::FatigueDynamicStateXia DeepCopy() const;
    void DeepCopy(const biorbd::muscles::FatigueDynamicStateXia& other);

    virtual void timeDerivativeState(
            const biorbd::muscles::StateDynamics &emg,
            const biorbd::muscles::Characteristics &characteristics
     );
protected:
    virtual void setType();

};

}}

#endif // BIORBD_MUSCLES_FATIGUE_DYNAMIC_STATE_XIA_H
