#ifndef S2MMUSCLEHILLTYPETHELENFATIGABLE_H
#define S2MMUSCLEHILLTYPETHELENFATIGABLE_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillTypeThelen.h"

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
        virtual void fatigueState(const std::shared_ptr<s2mMuscle> m, const s2mMuscleStateActual &EMG);
        virtual void computeFlCE(const s2mMuscleStateActual &EMG);



    protected:
        virtual void setType();

    private:

};

#endif // S2MMUSCLEHILLTYPETHELENFATIGABLE_H
