#ifndef S2MMUSCLEHILLTYPETHELENFATIGABLE_H
#define S2MMUSCLEHILLTYPETHELENFATIGABLE_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillTypeThelen.h"
    #include "s2mMuscleCaracteristics.h"
    #include "s2mMuscleFatigueParam.h"
    #include "s2mMuscleFatigueStateActualXia.h"

class BIORBD_API s2mMuscleHillTypeThelenFatigable : public s2mMuscleHillTypeThelen
{
    public:
        s2mMuscleHillTypeThelenFatigable(const s2mString& s= "");
        s2mMuscleHillTypeThelenFatigable(
                const s2mMuscleGeometry& g,
                const s2mMuscleCaracteristics& c,
                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                const s2mMuscleStateActual & s= s2mMuscleStateActual());
        s2mMuscleHillTypeThelenFatigable(
                const s2mString& n,
                const s2mMuscleGeometry& g,
                const s2mMuscleCaracteristics& c,
                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                const s2mMuscleStateActual & s= s2mMuscleStateActual());
        s2mMuscleHillTypeThelenFatigable(const s2mMuscle& m);
        s2mMuscleHillTypeThelenFatigable(const std::shared_ptr<s2mMuscle> m);

        ~s2mMuscleHillTypeThelenFatigable(){}
        virtual void timeDerivatedFatigueState(const s2mMuscleCaracteristics &c, const s2mMuscleStateActual &EMG);
        virtual void computeFlCE(const s2mMuscleStateActual &EMG);



    protected:
        virtual void setType();

    private:
        double m_fatigueRate;
        double m_recoveryRate;
        double m_developFactor;
        double m_recoverFactor;

        s2mMuscleFatigueStateActualXia m_fatigueState;

};

#endif // S2MMUSCLEHILLTYPETHELENFATIGABLE_H
