#ifndef S2M_WRAPPING_OBJECT_H
#define S2M_WRAPPING_OBJECT_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "s2mMusclePathChanger.h"
#include "Utils/String.h"

class s2mJoints;
class s2mGenCoord;
class BIORBD_API s2mWrappingObject : public s2mMusclePathChanger
{
    public:
        s2mWrappingObject(const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
                          const s2mString &name = "",  // Nom du noeud
                          const s2mString &parentName = "");
        virtual ~s2mWrappingObject() ;

        virtual s2mAttitude RT(s2mJoints &m, const s2mGenCoord& Q, const bool & = true) = 0;
        virtual void wrapPoints(const s2mAttitude&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0 ; // Premier et dernier points musculaire
        virtual void wrapPoints(s2mJoints&, const s2mGenCoord&, const s2mNodeMuscle&, const s2mNodeMuscle&, s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0; // Premier et dernier points musculaire
        virtual void wrapPoints(s2mNodeMuscle&, s2mNodeMuscle&, double* = nullptr) = 0; // Assume un appel déja faits

        // Set and get
        const s2mString& forme() const;

    protected:
        s2mString m_forme;

};

#endif // S2M_WRAPPING_OBJECT_H
