#ifndef S2MMUSCLEFATIGUESTATE_H
#define S2MMUSCLEFATIGUESTATE_H
#include "biorbdConfig.h"
class BIORBD_API s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueState(const double &mA = 0, const double &mF = 0, const double &mR = 1);
        virtual ~s2mMuscleFatigueState();

        // Set and Get
        virtual void setActiveFibers(const double &val);
        virtual void setFatiguedFibers(const double &val);
        virtual void setRestingFibers(const double &val);

        double activeFibers() const;
        double fatiguedFibers() const;
        double restingFibers() const;


    protected:
        double m_activeFibers;
        double m_fatiguedFibers;
        double m_restingFibers;

};

#endif // S2MMUSCLEFATIGUESTATE_H
