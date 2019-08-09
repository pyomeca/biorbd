#ifndef S2MMUSCLEHILLTYPESIMPLE_H
#define S2MMUSCLEHILLTYPESIMPLE_H

#include "biorbdConfig.h"
#include "s2mMuscleHillType.h"

class BIORBD_API s2mMuscleHillTypeSimple : public s2mMuscleHillType
{
public:
    s2mMuscleHillTypeSimple(const s2mString& s= "");
    s2mMuscleHillTypeSimple(const s2mMuscleGeometry& g,
                            const s2mMuscleCaracteristics& c,
                            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics());
    s2mMuscleHillTypeSimple(const s2mString& n,
                            const s2mMuscleGeometry& g,
                            const s2mMuscleCaracteristics& c,
                            const s2mMusclePathChangers & w= s2mMusclePathChangers(),
                            const s2mMuscleStateDynamics & s= s2mMuscleStateDynamics());
    s2mMuscleHillTypeSimple(const s2mMuscle& m);
    s2mMuscleHillTypeSimple(const std::shared_ptr<s2mMuscle> m);
    virtual ~s2mMuscleHillTypeSimple();

    virtual const std::vector<std::shared_ptr<s2mMuscleForce>>& force(const s2mMuscleStateDynamics &emg);
protected:
    double multiplyCaractByActivationAndForce(const s2mMuscleStateDynamics &emg); // Voir dans la fonction pour descriptif
    virtual void setType();

};

#endif // S2MMUSCLEHILLTYPESIMPLE_H
