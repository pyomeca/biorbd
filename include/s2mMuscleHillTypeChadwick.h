#ifndef S2MMUSCLEHILLTYPECHADWICK_H
#define S2MMUSCLEHILLTYPECHADWICK_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillType.h"

class BIORBD_API s2mMuscleHillTypeChadwick : public s2mMuscleHillType
{
    public:
    s2mMuscleHillTypeChadwick(const s2mString& s= "") : s2mMuscleHillType(s){setType();}
        s2mMuscleHillTypeChadwick(const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
            s2mMuscleHillType(g,c,w,s){setType();}
        s2mMuscleHillTypeChadwick(const s2mString& n,
                                const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
            s2mMuscleHillType(n,g,c,w,s){setType();}
        ~s2mMuscleHillTypeChadwick(){}

    protected:

    private:

};

#endif // S2MMUSCLEHILLTYPECHADWICK_H
