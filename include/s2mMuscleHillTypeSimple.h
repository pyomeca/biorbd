#ifndef S2MMUSCLEHILLTYPESIMPLE_H
#define S2MMUSCLEHILLTYPESIMPLE_H
    #include "biorbdConfig.h"
    #include "s2mMuscleHillType.h"

class BIORBD_API s2mMuscleHillTypeSimple : public s2mMuscleHillType
{
    public:
    s2mMuscleHillTypeSimple(const s2mString& s= "") : s2mMuscleHillType(s){setType();}
        s2mMuscleHillTypeSimple(const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
        s2mMuscleHillType(g,c,w,s){setType();}
        s2mMuscleHillTypeSimple(const s2mString& n,
                                const s2mMuscleGeometry& g,
                                const s2mMuscleCaracteristics& c,
                                const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                                const s2mMuscleStateActual & s= s2mMuscleStateActual()) :
        s2mMuscleHillType(n,g,c,w,s){setType();}
        ~s2mMuscleHillTypeSimple(){}
    std::vector<boost::shared_ptr<s2mMuscleForce> > force(const s2mMuscleStateActual &EMG);
    protected:
        double multiplyCaractByActivationAndForce(const s2mMuscleStateActual &EMG); // Voir dans la fonction pour descriptif

    private:

};

#endif // S2MMUSCLEHILLTYPESIMPLE_H
