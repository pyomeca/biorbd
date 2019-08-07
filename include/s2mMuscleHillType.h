#ifndef S2M_MUSCLE_HILL_TYPE_H
#define S2M_MUSCLE_HILL_TYPE_H

#include "biorbdConfig.h"
#include "s2mMuscle.h"

class BIORBD_API s2mMuscleHillType : public s2mMuscle
{
    public:
        s2mMuscleHillType(const s2mString& = "");
        s2mMuscleHillType(const s2mMuscleGeometry&,
                          const s2mMuscleCaracteristics&,
                          const s2mMusclePathChangers & = s2mMusclePathChangers(),
                          const s2mMuscleStateDynamics & = s2mMuscleStateDynamics());
        s2mMuscleHillType(const s2mString&,
                          const s2mMuscleGeometry&,
                          const s2mMuscleCaracteristics&,
                          const s2mMusclePathChangers & = s2mMusclePathChangers(),
                          const s2mMuscleStateDynamics & = s2mMuscleStateDynamics());
        s2mMuscleHillType(const s2mMuscle& m);
        s2mMuscleHillType(const std::shared_ptr<s2mMuscle> m);
        virtual ~s2mMuscleHillType();


        virtual const std::vector<std::shared_ptr<s2mMuscleForce>>& force(s2mJoints& model, const s2mGenCoord& Q, const s2mGenCoord& Qdot, const s2mMuscleStateDynamics& emg, const int updateKin = 2); // Compute muscle force
        virtual const std::vector<std::shared_ptr<s2mMuscleForce>>& force(s2mJoints& model, const s2mGenCoord& Q, const s2mMuscleStateDynamics& emg, const int updateKin = 2); // Compute muscle force
        virtual const std::vector<std::shared_ptr<s2mMuscleForce>>& force(const s2mMuscleStateDynamics& emg); // Compute muscle force (assume updateOrientations has been done)

        // Get individual forces
        double FlCE(const s2mMuscleStateDynamics& EMG);
        double FlPE();
        double FvCE();
        double damping();

    protected:
        virtual void setType();
        virtual void setForce();

        // Étapes intermédiaires du calcul de la force
        virtual void computeDamping(); // Force du ressort
        virtual void computeFlCE(const s2mMuscleStateDynamics &EMG); // ForceLongueur element contractile
        virtual void computeFvCE(); // ForceVitesse element contractile
        virtual void computeFlPE(); // ForceLongueur element passif
        virtual void computeForce(const s2mMuscleStateDynamics &EMG); // Calcul des forces
        virtual double multiplyCaractByActivationAndForce(const s2mMuscleStateDynamics &emg); // Voir dans la fonction pour descriptif
        virtual s2mMuscleStateDynamics normalizeEMG(const s2mMuscleStateDynamics& emg);

        // Attributs intermédiaires lors du calcul de la force
        double m_damping; // Force du ressort
        double m_FlCE; // Force-Longueur contractile element
        double m_FlPE; // Force-Longueur passive element
        double m_FvCE; // Force-vitesse contractile element

        // Déclaration de plusieurs constantes
        double m_cste_FlCE_1; // constante utilisé dans la FlCE
        double m_cste_FlCE_2; // constante utilisé dans la FlCE
        double m_cste_FvCE_1; // constante utisisé dans la FvCE
        double m_cste_FvCE_2; // constante utisisé dans la FvCE
        double m_cste_FlPE_1; // constante utisisé dans la FlPE
        double m_cste_FlPE_2; // constante utisisé dans la FlPE
        double m_cste_forceExcentriqueMultiplier; // Constante utilisée pour ForceVitesse
        double m_cste_damping; // Parametre utilisé dans le Damping
        double m_cste_vitesseRaccourMax; // Vitesse de raccourcissement maximale

};

#endif // S2M_MUSCLE_HILL_TYPE_H
