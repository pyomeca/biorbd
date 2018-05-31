#ifndef S2MMUSCLESTATE_H
#define S2MMUSCLESTATE_H
#include "biorbdConfig.h"
class BIORBD_API s2mMuscleState
{
    public:
        s2mMuscleState(const double &e = 0, const double &a = 0);
        ~s2mMuscleState();

        // Set and Get
        virtual void setExcitation(const double &val);
        virtual void setActivation(const double &val);

        double excitation() const;
        double activation() const;


    protected:
        double m_excitation;
        double m_activation;

};

#endif // S2MMUSCLESTATE_H
