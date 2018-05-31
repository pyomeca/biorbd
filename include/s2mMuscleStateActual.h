#ifndef S2MMUSCLESTATEACTUAL_H
#define S2MMUSCLESTATEACTUAL_H
#include "biorbdConfig.h"
#include "s2mMuscleState.h"
#include "s2mMuscleStateMax.h"
#include "s2mMuscleCaracteristics.h"

class BIORBD_API s2mMuscleStateActual : public s2mMuscleState
{
    public:
        s2mMuscleStateActual(const double &e = 0, const double &a = 0);
        ~s2mMuscleStateActual();

        virtual void setExcitation(const double &val);
        virtual void setActivation(const double &val);

        double excitationNorm(const s2mMuscleState &max);
        double excitationNorm() const {return m_excitationNorm;} // Retourne la derniere excitation normalisee
        void setExcitationNorm(double val) {m_excitationNorm = val;} // Retourne la derniere excitation normalisee
        double previousActivation() const { return m_previousActivation; }
        double previousExcitation() const { return m_previousExcitation; }

        virtual double timeDerivativeActivation(double, double, const s2mMuscleCaracteristics&, const bool =false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        virtual double timeDerivativeActivation(const s2mMuscleStateActual&, const s2mMuscleCaracteristics&, const bool =false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        virtual double timeDerivativeActivation(const s2mMuscleCaracteristics&, const bool =false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        virtual double timeDerivativeActivation() {return m_activationDot;} // Retourne la derni`ere valeur


    protected:
        double m_excitationNorm;
        double m_previousExcitation;
        double m_previousActivation;
        double m_activationDot;
};

#endif // S2MMUSCLESTATEACTUAL_H
