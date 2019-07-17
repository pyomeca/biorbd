#ifndef S2MMUSCLEFATIGUESTATEACTUAL_H
#define S2MMUSCLEFATIGUESTATEACTUAL_H
#include "biorbdConfig.h"
#include "s2mMuscleFatigueState.h"
#include "s2mMuscleCaracteristics.h"
#include "s2mMuscleFatigueParam.h"

class BIORBD_API s2mMuscleFatigueStateActual : public s2mMuscleFatigueState
{
    public:
        s2mMuscleFatigueStateActual(const double &mA = 0, const double &mF = 0, const double &mR = 1);
        ~s2mMuscleFatigueStateActual();

        virtual void setActiveFibers(const double &val);
        virtual void setFatiguedFibers(const double &val);
        virtual void setRestingFibers(const double &val);

        double previousActiveFibers() const { return m_previousActiveFibers; }
        double previousFatiguedFibers() const { return m_previousFatiguedFibers; }
        double previousRestingFibers() const { return m_previousRestingFibers; }

        //virtual double timeDerivativeActivation(double, double, const s2mMuscleCaracteristics&, const bool =false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        //virtual double timeDerivativeActivation(const s2mMuscleStateActual&, const s2mMuscleCaracteristics&, const bool =false); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        virtual std::vector<double> timeDerivativeState(const s2mMuscleCaracteristics&); // Fonction de calcul de la vitesse d'activation en fonction de l'excitation et de l'activation
        //virtual double timeDerivativeActivation() {return m_activationDot;} // Retourne la derni`ere valeur


    protected:
        double m_previousActiveFibers;
        double m_previousFatiguedFibers;
        double m_previousRestingFibers;
        double m_activeFibersDot;
        double m_fatiguedFibersDot;
        double m_restingFibersDot;
};

#endif // S2MMUSCLEFATIGUESTATEACTUAL_H
