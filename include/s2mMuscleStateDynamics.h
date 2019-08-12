#ifndef S2M_MUSCLE_STATE_DYNAMICS_H
#define S2M_MUSCLE_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "s2mMuscleState.h"

class s2mMuscleCaracteristics;
class BIORBD_API s2mMuscleStateDynamics : public s2mMuscleState
{
public:
    s2mMuscleStateDynamics(
            const double &e = 0,
            const double &a = 0);
    virtual ~s2mMuscleStateDynamics();

    virtual void setExcitation(const double &val);
    virtual void setActivation(const double &val);

    double excitationNorm(const s2mMuscleState &max);
    double excitationNorm() const; // Retourne la derniere excitation normalisee
    void setExcitationNorm(double val); // Retourne la derniere excitation normalisee
    double previousActivation() const;
    double previousExcitation() const;

    virtual double timeDerivativeActivation(
            double excitation,
            double activation,
            const s2mMuscleCaracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const s2mMuscleStateDynamics& state,
            const s2mMuscleCaracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const s2mMuscleCaracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(); // Retourne la derniere valeur

protected:
    double m_excitationNorm;
    double m_previousExcitation;
    double m_previousActivation;
    double m_activationDot;

};

#endif // S2M_MUSCLE_STATE_DYNAMICS_H
