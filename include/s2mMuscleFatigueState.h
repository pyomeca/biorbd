#ifndef S2MMUSCLEFATIGUESTATE_H
#define S2MMUSCLEFATIGUESTATE_H
#include "biorbdConfig.h"
#include "s2mError.h"

class BIORBD_API s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueState(
                double active = 0,
                double fatigued = 0,
                double resting = 1);

        virtual ~s2mMuscleFatigueState();

        // Set and Get
        virtual void setState(
                double active,
                double fatigued,
                double resting);

        double activeFibers() const;
        double fatiguedFibers() const;
        double restingFibers() const;

        std::string getType() const;
    protected:
        double m_activeFibers;
        double m_fatiguedFibers;
        double m_restingFibers;

        virtual void setType();
        std::string m_type; // type of the muscle fatigue

};

#endif // S2MMUSCLEFATIGUESTATE_H
