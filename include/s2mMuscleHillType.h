#ifndef S2MMUSCLEHILLTYPE_H
#define S2MMUSCLEHILLTYPE_H
    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include "s2mMuscle.h"
    #include "s2mMuscleForceFromOrigin.h"
    #include "s2mMuscleForceFromInsertion.h"
    #include "s2mGenCoord.h"
    #include <math.h>
    #include <memory>

class BIORBD_API s2mMuscleHillType : public s2mMuscle
{
    public:
        s2mMuscleHillType(const s2mString& = "");
        s2mMuscleHillType(const s2mMuscleGeometry&,
                          const s2mMuscleCaracteristics&,
                          const s2mMusclePathChangers & = s2mMusclePathChangers(),
                          const s2mMuscleStateActual & = s2mMuscleStateActual());
        s2mMuscleHillType(const s2mString&,
                          const s2mMuscleGeometry&,
                          const s2mMuscleCaracteristics&,
                          const s2mMusclePathChangers & = s2mMusclePathChangers(),
                          const s2mMuscleStateActual & = s2mMuscleStateActual());
        s2mMuscleHillType(const s2mMuscle& m);
        s2mMuscleHillType(const std::shared_ptr<s2mMuscle> m);
        virtual ~s2mMuscleHillType();


        virtual std::vector<std::shared_ptr<s2mMuscleForce> > force(s2mJoints& , const s2mGenCoord&, const s2mGenCoord&, const s2mMuscleStateActual&, const int = 2); // Compute muscle force
        virtual std::vector<std::shared_ptr<s2mMuscleForce> > force(s2mJoints& , const s2mGenCoord&, const s2mMuscleStateActual&, const int = 2){s2mError::s2mAssert(0, "Hill type needs velocity"); std::vector<std::shared_ptr<s2mMuscleForce> > dummy; return dummy;} // Compute muscle force
        virtual std::vector<std::shared_ptr<s2mMuscleForce> > force(const s2mMuscleStateActual&); // Compute muscle force (assume updateOrientations has been done)

        // Get individual forces
        double FlCE(const s2mMuscleStateActual& EMG);
        double FlPE();
        double FvCE();
        double damping();

    protected:
        virtual void setType();
        virtual void setForce();

        // Étapes intermédiaires du calcul de la force
        virtual void computeDamping(); // Force du ressort
        virtual void computeFlCE(const s2mMuscleStateActual &EMG); // ForceLongueur element contractile
        virtual void computeFvCE(); // ForceVitesse element contractile
        virtual void computeFlPE(); // ForceLongueur element passif
        virtual void computeForce(const s2mMuscleStateActual &EMG); // Calcul des forces
        virtual double multiplyCaractByActivationAndForce(const s2mMuscleStateActual &EMG); // Voir dans la fonction pour descriptif
        virtual s2mMuscleStateActual normalizeEMG(s2mMuscleStateActual EMG);

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
    private:
        void initiateForcePtr(){}

};

#endif // S2MMUSCLEHILLTYPE_H
