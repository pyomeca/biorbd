#ifndef S2MMUSCLE_H
#define S2MMUSCLE_H
    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include "s2mMuscleCompound.h"
    #include "s2mJoints.h"
    #include "s2mMusclePathChangers.h"
    #include "s2mMuscleStateActual.h"
    #include "s2mMuscleStateActualBuchanan.h"
    #include "s2mGenCoord.h"

class BIORBD_API s2mMuscle : public s2mMuscleCompound
{
    public:
        s2mMuscle(const s2mString& = "", // Nom du muscle
                  const s2mMuscleGeometry& = s2mMuscleGeometry(), // Position origine/insertion
                  const s2mMuscleCaracteristics& = s2mMuscleCaracteristics(), // Caractéristiques du muscle
                  const s2mMusclePathChangers& = s2mMusclePathChangers(), // Set de wrapping objects
                  const s2mMuscleStateActual& = s2mMuscleStateActual() // Set d'un état actuel au départ
                  );
        s2mMuscle(const s2mMuscle& m);
        virtual ~s2mMuscle();

        // Get and set
        virtual double length(s2mJoints&, const s2mGenCoord&, int = 2);
        virtual double musculoTendonLength(s2mJoints&, const s2mGenCoord&, int = 2);
        virtual double velocity(s2mJoints&, const s2mGenCoord&, const s2mGenCoord&, const bool = true);
        virtual void updateOrientations(s2mJoints &m, const s2mGenCoord &Q, int updateKin = 2); // Update de la position de ce muscle
        virtual void updateOrientations(s2mJoints &m, const s2mGenCoord &Q, const s2mGenCoord &Qdot, int updateKin = 2); // Update de la position de ce muscle
        virtual void updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix& jacoPointsInGlobal); // Update de la position de ce muscle
        virtual void updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix& jacoPointsInGlobal, const s2mGenCoord &Qdot); // Update de la position de ce muscle

        virtual s2mMuscleGeometry position() const;
        virtual const s2mMuscleCaracteristics& caract() const;
        virtual void setPosition(const s2mMuscleGeometry &val);
        virtual void setCaract(const s2mMuscleCaracteristics &val);
        std::vector<s2mNodeMuscle> musclesPointsInGlobal(s2mJoints &j, const s2mGenCoord &Q,const bool updateKin = true);
        virtual void forceIsoMax(double);

        // Get and set
        virtual void setState(const s2mMuscleStateActual &s);
        virtual const s2mMuscleStateActual& state() const;
        virtual s2mMuscleStateActual& state_nonConst() const;
        virtual double activationDot(const s2mMuscleStateActual &s, const bool =false);
    protected:
        s2mMuscleGeometry m_position;
        s2mMuscleCaracteristics m_caract;
        s2mMuscleStateActual * m_state;

    private:
};

#endif // S2MMUSCLE_H
