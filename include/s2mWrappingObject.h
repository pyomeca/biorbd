#ifndef S2MWRAPPINGOBJECT_H
#define S2MWRAPPINGOBJECT_H
    #include "biorbdConfig.h"
    #include "s2mJoints.h"
    #include "s2mAttitude.h"
    #include "s2mString.h"
    #include "s2mNodeMuscle.h"
    #include "s2mMusclePathChanger.h"
    #include "s2mGenCoord.h"

class s2mJoints;
class BIORBD_API s2mWrappingObject : public s2mMusclePathChanger
{
    public:
        s2mWrappingObject(const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
                          const s2mString &name = "",  // Nom du noeud
                          const s2mString &parentName = "");
        ~s2mWrappingObject() ;

        virtual s2mAttitude RT(s2mJoints &m, const s2mGenCoord& Q, const bool & = true) = 0;
        virtual void wrapPoints(const s2mAttitude&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0 ; // Premier et dernier points musculaire
        virtual void wrapPoints(s2mJoints&, const s2mGenCoord&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0; // Premier et dernier points musculaire
        virtual void wrapPoints(s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0; // Assume un appel déja faits

        // Set and get
        virtual s2mString type() {return "wrapping";}
        virtual s2mString forme() const = 0;

    protected:
    private:
};

#endif // S2MWRAPPINGOBJECT_H
