#ifndef BIORBD_MUSCLES_HILL_TYPE_H
#define BIORBD_MUSCLES_HILL_TYPE_H

#include "biorbdConfig.h"
#include "Muscles/Muscle.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillType : public biorbd::muscles::Muscle
{
public:
    HillType(const biorbd::utils::String& = "");
    HillType(
            const biorbd::muscles::Geometry&,
            const biorbd::muscles::Caracteristics&,
            const biorbd::muscles::PathChangers & = biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & = biorbd::muscles::StateDynamics());
    HillType(
            const biorbd::utils::String&,
            const biorbd::muscles::Geometry&,
            const biorbd::muscles::Caracteristics&,
            const biorbd::muscles::PathChangers & = biorbd::muscles::PathChangers(),
            const biorbd::muscles::StateDynamics & = biorbd::muscles::StateDynamics());
    HillType(const biorbd::muscles::Muscle& m);
    HillType(const std::shared_ptr<biorbd::muscles::Muscle> m);
    virtual ~HillType();


    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            const int updateKin = 2); // Compute muscle force
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            const int updateKin = 2); // Compute muscle force
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(const biorbd::muscles::StateDynamics& emg); // Compute muscle force (assume updateOrientations has been done)

    // Get individual forces
    double FlCE(const biorbd::muscles::StateDynamics& EMG);
    double FlPE();
    double FvCE();
    double damping();

protected:
    virtual void setType();
    virtual void setForce();

    // Étapes intermédiaires du calcul de la force
    virtual void computeDamping(); // Force du ressort
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG); // ForceLongueur element contractile
    virtual void computeFvCE(); // ForceVitesse element contractile
    virtual void computeFlPE(); // ForceLongueur element passif
    virtual void computeForce(const biorbd::muscles::StateDynamics &EMG); // Calcul des forces
    virtual double multiplyCaractByActivationAndForce(const biorbd::muscles::StateDynamics &emg); // Voir dans la fonction pour descriptif
    virtual biorbd::muscles::StateDynamics normalizeEMG(const biorbd::muscles::StateDynamics& emg);

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

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_H
