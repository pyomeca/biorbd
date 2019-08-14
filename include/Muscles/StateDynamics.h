#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "Muscles/State.h"

namespace biorbd { namespace muscles {

class Caracteristics;
class BIORBD_API StateDynamics : public biorbd::muscles::State
{
public:
    StateDynamics(
            const double &e = 0,
            const double &a = 0);
    virtual ~StateDynamics();

    virtual void setExcitation(const double &val);
    virtual void setActivation(const double &val);

    double excitationNorm(const State &max);
    double excitationNorm() const; // Retourne la derniere excitation normalisee
    void setExcitationNorm(double val); // Retourne la derniere excitation normalisee
    double previousActivation() const;
    double previousExcitation() const;

    virtual double timeDerivativeActivation(
            double excitation,
            double activation,
            const Caracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const StateDynamics& state,
            const Caracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(
            const Caracteristics& caract,
            const bool alreadyNormalized = false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
    virtual double timeDerivativeActivation(); // Retourne la derniere valeur

protected:
    double m_excitationNorm;
    double m_previousExcitation;
    double m_previousActivation;
    double m_activationDot;

};

}}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_H
