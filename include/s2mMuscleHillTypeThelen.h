#ifndef S2MMUSCLEHILLTYPETHELEN_H
#define S2MMUSCLEHILLTYPETHELEN_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillType.h"

class BIORBD_API s2mMuscleHillTypeThelen : public s2mMuscleHillType
{
    public:
    	s2mMuscleHillTypeThelen(const s2mString& s= "") : s2mMuscleHillType(s){setType();}
        s2mMuscleHillTypeThelen(const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual());

        s2mMuscleHillTypeThelen(const s2mString& n,
                                const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual());

        ~s2mMuscleHillTypeThelen(){}
        virtual void computeFlCE(const s2mMuscleStateActual &EMG);


    protected:

    private:

};

#endif // S2MMUSCLEHILLTYPETHELEN_H
