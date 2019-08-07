#ifndef S2M_MUSCLE_STATE_DYNAMICS_BUCHANAN_H
#define S2M_MUSCLE_STATE_DYNAMICS_BUCHANAN_H

#include "biorbdConfig.h"
#include "s2mMuscleStateDynamics.h"

class BIORBD_API s2mMuscleStateDynamicsBuchanan : public s2mMuscleStateDynamics
{
    public:
        s2mMuscleStateDynamicsBuchanan(const double &neuralCommand = 0, const double &excitation = 0);
        ~s2mMuscleStateDynamicsBuchanan();

        virtual double timeDerivativeExcitation(const s2mMuscleCaracteristics &caract, const bool alreadyNormalized);
        virtual void setExcitation(const double &val);
        void setNeuralCommand(const double &val);
        void shapeFactor(double m_shape_factor);
        double shapeFactor();
        double activation();

    protected:
        double m_neuralCommand;
        double m_shapeFactor; //Buchanan2004, le 22 mars 2018
        double m_excitationDot;

};

#endif // S2M_MUSCLE_STATE_DYNAMICS_BUCHANAN_H
