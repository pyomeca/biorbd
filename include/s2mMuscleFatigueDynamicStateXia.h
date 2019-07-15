#ifndef S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
#define S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
#include "biorbdConfig.h"
#include "s2mMuscleFatigueState.h"
#include "s2mMuscleCaracteristics.h"
#include "s2mMuscleFatigueParam.h"
#include "s2mMuscleStateActual.h"
#include "s2mVector.h"

// Actual pour current
class BIORBD_API s2mMuscleFatigueStateActualXia : public s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueStateActualXia(const double &mA = 0, const double &mF = 0, const double &mR = 1);
        ~s2mMuscleFatigueStateActualXia();

        // TODO refactor in file
        double previousActiveFibers() const;
        double previousFatiguedFibers() const;
        double previousRestingFibers() const;

        virtual s2mVector timeDerivativeState(
                const s2mMuscleStateActual &EMG,
                const s2mMuscleCaracteristics &c
         );



    protected:
        double m_previousActiveFibers;
        double m_previousFatiguedFibers;
        double m_previousRestingFibers;
        double m_activeFibersDot;
        double m_fatiguedFibersDot;
        double m_restingFibersDot;
};

#endif // S2MMUSCLEFATIGUEDYNAMICSTATEXIA_H
