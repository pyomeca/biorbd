#ifndef S2MMUSCLEFATIGUESTATE_H
#define S2MMUSCLEFATIGUESTATE_H
#include "biorbdConfig.h"
#include "s2mError.h"

class BIORBD_API s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueState(const double &mA = 0, const double &mF = 0, const double &mR = 1);
        virtual ~s2mMuscleFatigueState();

        // Set and Get
        virtual void setState(
                const double &mA = 0,
                const double &mF = 0,
                const double &mR = 1
                );


        double activeFibers() const;
        double fatiguedFibers() const;
        double restingFibers() const;


    protected:
        double m_activeFibers;
        double m_fatiguedFibers;
        double m_restingFibers;

};

#endif // S2MMUSCLEFATIGUESTATE_H
