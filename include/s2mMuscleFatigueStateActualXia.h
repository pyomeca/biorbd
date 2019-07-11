#ifndef S2MMUSCLEFATIGUESTATEACTUALXIA_H
#define S2MMUSCLEFATIGUESTATEACTUALXIA_H
#include "biorbdConfig.h"
#include "s2mMuscleFatigueState.h"
#include "s2mMuscleCaracteristics.h"
#include "s2mMuscleFatigueParam.h"
#include "s2mMuscleStateActual.h"

class BIORBD_API s2mMuscleFatigueStateActualXia : public s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueStateActualXia(const double &mA = 0, const double &mF = 0, const double &mR = 1);
        ~s2mMuscleFatigueStateActualXia();

        virtual void setActiveFibers(const double &val);
        virtual void setFatiguedFibers(const double &val);
        virtual void setRestingFibers(const double &val);

        double previousActiveFibers() const { return m_previousActiveFibers; }
        double previousFatiguedFibers() const { return m_previousFatiguedFibers; }
        double previousRestingFibers() const { return m_previousRestingFibers; }

        virtual std::vector<double> timeDerivativeState(
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

#endif // S2MMUSCLEFATIGUESTATEACTUALXIA_H
