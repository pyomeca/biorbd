#ifndef S2MMUSCLEHILLTYPESCHUTTE_H
#define S2MMUSCLEHILLTYPESCHUTTE_H
    #include "s2mMuscleHillType.h"

class s2mMuscleHillTypeSchutte : public s2mMuscleHillType
{
    public:
    s2mMuscleHillTypeSchutte(const s2mString& s= "") : s2mMuscleHillType(s){setType();}
        s2mMuscleHillTypeSchutte(const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
            s2mMuscleHillType(g,c,w,s){setType();}
        s2mMuscleHillTypeSchutte(const s2mString& n,
                                const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
            s2mMuscleHillType(n,g,c,w,s){setType();}
        ~s2mMuscleHillTypeSchutte(){}

    protected:

    private:

};

#endif // S2MMUSCLEHILLTYPESCHUTTE_H
