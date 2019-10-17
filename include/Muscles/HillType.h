#ifndef BIORBD_MUSCLES_HILL_TYPE_H
#define BIORBD_MUSCLES_HILL_TYPE_H

#include "biorbdConfig.h"
#include "Muscles/Muscle.h"

namespace biorbd {
namespace muscles {

class BIORBD_API HillType : public biorbd::muscles::Muscle
{
public:
    HillType();
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics);
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers);
    HillType(
            const biorbd::utils::String& name,
            const biorbd::muscles::Geometry& geometry,
            const biorbd::muscles::Characteristics& characteristics,
            const biorbd::muscles::PathChangers& pathChangers,
            const biorbd::muscles::StateDynamics& dynamicState);
    HillType(
            const biorbd::muscles::Muscle& muscle);
    HillType(
            const std::shared_ptr<biorbd::muscles::Muscle> muscle);
    biorbd::muscles::HillType DeepCopy() const;
    void DeepCopy(const biorbd::muscles::HillType& other);

    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            const biorbd::muscles::StateDynamics& emg);
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);
    virtual const std::vector<std::shared_ptr<biorbd::muscles::Force>>& force(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::muscles::StateDynamics& emg,
            int updateKin = 2);

    // Get individual forces
    double FlCE(const biorbd::muscles::StateDynamics& EMG);
    double FlPE();
    double FvCE();
    double damping();

protected:
    virtual void setType();

    // Étapes intermédiaires du calcul de la force
    virtual void computeDamping(); // Force du ressort
    virtual void computeFlCE(const biorbd::muscles::StateDynamics &EMG); // ForceLongueur element contractile
    virtual void computeFvCE(); // ForceVitesse element contractile
    virtual void computeFlPE(); // ForceLongueur element passif
    virtual double getForceFromActivation(const biorbd::muscles::State &emg); // Voir dans la fonction pour descriptif
    virtual biorbd::muscles::StateDynamics normalizeEMG(const biorbd::muscles::StateDynamics& emg);

    // Attributs intermédiaires lors du calcul de la force
    std::shared_ptr<double> m_damping; // Force du ressort
    std::shared_ptr<double> m_FlCE; // Force-Longueur contractile element
    std::shared_ptr<double> m_FlPE; // Force-Longueur passive element
    std::shared_ptr<double> m_FvCE; // Force-vitesse contractile element

    // Déclaration de plusieurs constantes
    std::shared_ptr<double> m_cste_FlCE_1; // constante utilisé dans la FlCE
    std::shared_ptr<double> m_cste_FlCE_2; // constante utilisé dans la FlCE
    std::shared_ptr<double> m_cste_FvCE_1; // constante utisisé dans la FvCE
    std::shared_ptr<double> m_cste_FvCE_2; // constante utisisé dans la FvCE
    std::shared_ptr<double> m_cste_FlPE_1; // constante utisisé dans la FlPE
    std::shared_ptr<double> m_cste_FlPE_2; // constante utisisé dans la FlPE
    std::shared_ptr<double> m_cste_forceExcentriqueMultiplier; // Constante utilisée pour ForceVitesse
    std::shared_ptr<double> m_cste_damping; // Parametre utilisé dans le Damping
    std::shared_ptr<double> m_cste_vitesseRaccourMax; // Vitesse de raccourcissement maximale

};

}}

#endif // BIORBD_MUSCLES_HILL_TYPE_H
