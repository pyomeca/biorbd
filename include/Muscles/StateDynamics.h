#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "Muscles/State.h"

namespace biorbd {
namespace muscles {
class Characteristics;

class BIORBD_API StateDynamics : public biorbd::muscles::State
{
public:
    StateDynamics(
            double excitation = 0,
            double activation = 0);
    StateDynamics(const biorbd::muscles::StateDynamics& other);
    virtual ~StateDynamics();
    biorbd::muscles::StateDynamics DeepCopy() const;
    void DeepCopy(const biorbd::muscles::StateDynamics& other);

    virtual void setExcitation(double val);
    virtual void setActivation(double val);

    double excitationNorm(const State &max);
    double excitationNorm() const; // Retourne la derniere excitation normalisee
    void setExcitationNorm(double val); // Retourne la derniere excitation normalisee
    double previousActivation() const;
    double previousExcitation() const;

    virtual double timeDerivativeActivation(
            double excitation,
            double activation,
            const Characteristics& characteristics,
            bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const StateDynamics& state,
            const Characteristics& characteristics,
            bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const Characteristics& characteristics,
            bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(); // Retourne la derniere valeur

protected:
    virtual void setType();
    std::shared_ptr<double> m_excitationNorm;
    std::shared_ptr<double> m_previousExcitation;
    std::shared_ptr<double> m_previousActivation;
    std::shared_ptr<double> m_activationDot;

};

}}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_H
