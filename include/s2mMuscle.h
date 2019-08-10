#ifndef S2M_MUSCLE_H
#define S2M_MUSCLE_H

#include "biorbdConfig.h"
#include "Utils/String.h"
#include "s2mMuscleCompound.h"
#include "s2mMuscleGeometry.h"
#include "s2mMuscleCaracteristics.h"
#include "s2mMuscleStateDynamics.h"

class BIORBD_API s2mMuscle : public s2mMuscleCompound
{
    public:
        s2mMuscle(const s2mString& = "", // Nom du muscle
                  const s2mMuscleGeometry& = s2mMuscleGeometry(), // Position origine/insertion
                  const s2mMuscleCaracteristics& = s2mMuscleCaracteristics(), // Caractéristiques du muscle
                  const s2mMusclePathChangers& = s2mMusclePathChangers(), // Set de wrapping objects
                  const s2mMuscleStateDynamics& = s2mMuscleStateDynamics() // Set d'un état actuel au départ
                  );
        s2mMuscle(const s2mMuscle& m);
        virtual ~s2mMuscle();

        // Get and set
        double length(s2mJoints&, const s2mGenCoord&, int = 2);
        double musculoTendonLength(s2mJoints&, const s2mGenCoord&, int = 2);
        double velocity(s2mJoints&, const s2mGenCoord&, const s2mGenCoord&, const bool = true);
        void updateOrientations(s2mJoints &m, const s2mGenCoord &Q, int updateKin = 2); // Update de la position de ce muscle
        void updateOrientations(s2mJoints &m, const s2mGenCoord &Q, const s2mGenCoord &Qdot, int updateKin = 2); // Update de la position de ce muscle
        void updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix& jacoPointsInGlobal); // Update de la position de ce muscle
        void updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix& jacoPointsInGlobal, const s2mGenCoord &Qdot); // Update de la position de ce muscle

        const s2mMuscleGeometry& position() const;
        const s2mMuscleCaracteristics& caract() const;
        void setPosition(const s2mMuscleGeometry &val);
        void setCaract(const s2mMuscleCaracteristics &val);
        const std::vector<s2mNodeMuscle>& musclesPointsInGlobal(s2mJoints &j, const s2mGenCoord &Q,const bool updateKin = true);
        void forceIsoMax(double);

        // Get and set
        void setState(const s2mMuscleStateDynamics &s);
        const s2mMuscleStateDynamics& state() const;
        s2mMuscleStateDynamics& state_nonConst() const;
        double activationDot(const s2mMuscleStateDynamics &s, const bool =false);
    protected:
        s2mMuscleGeometry m_position;
        s2mMuscleCaracteristics m_caract;
        s2mMuscleStateDynamics * m_state;

};

#endif // S2M_MUSCLE_H
