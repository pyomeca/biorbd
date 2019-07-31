#ifndef S2M_MUSCLE_FATIGABLE_H
#define S2M_MUSCLE_FATIGABLE_H

#include <memory>
#include "biorbdConfig.h"
#include "s2mMuscleFatigueDynamicStateXia.h"
#include "s2mMuscleFatigueState.h"
#include "s2mMuscleStateActual.h"

class BIORBD_API s2mMuscleFatigable
{
public:
    s2mMuscleFatigable(const s2mString& dynamicFatigueType);
    s2mMuscleFatigable(const s2mMuscle& m);
    s2mMuscleFatigable(const std::shared_ptr<s2mMuscle> m);
    virtual ~s2mMuscleFatigable() = 0;

    virtual void computeTimeDerivativeState(const s2mMuscleStateActual& EMG);

    std::shared_ptr<s2mMuscleFatigueState> fatigueState();
    virtual void fatigueState(double active, double fatigued, double resting);

protected:
    std::shared_ptr<s2mMuscleFatigueState> m_fatigueState;

};

#endif // S2M_MUSCLE_FATIGABLE_H
