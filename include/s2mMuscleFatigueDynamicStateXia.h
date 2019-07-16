#ifndef S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
#define S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
#include "biorbdConfig.h"
#include "s2mMuscleFatigueDynamicState.h"
#include "s2mMuscleFatigueState.h"
#include "s2mMuscleCaracteristics.h"
#include "s2mMuscleFatigueParam.h"
#include "s2mMuscleStateActual.h"
#include "s2mVector.h"

class BIORBD_API s2mMuscleFatigueDynamicStateXia : public s2mMuscleFatigueDynamicState
{
    public:
        s2mMuscleFatigueDynamicStateXia(
                double active = 0,
                double fatigued = 0,
                double resting = 1);

        virtual s2mVector timeDerivativeState(
                const s2mMuscleStateActual &EMG,
                const s2mMuscleCaracteristics &caract
         );
    protected:
        virtual void setType();

};

#endif // S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
