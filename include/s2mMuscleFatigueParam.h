#ifndef S2MMUSCLEFATIGUEPARAM_H
#define S2MMUSCLEFATIGUEPARAM_H
    #include "biorbdConfig.h"

class BIORBD_API s2mMuscleFatigueParam
{
    public:
        s2mMuscleFatigueParam(
                double fatigueRate = 0,
                double recoveryRate = 0,
                double developFactor = 0,
                double recoveryFactor = 0);

        // Get and Set
        double fatigueRate() const;
        double recoveryRate() const;
        double developFactor() const;
        double recoveryFactor() const;

        void fatigueRate(double fatigueRate);
        void recoveryRate(double recoveryRate);
        void developFactor(double developFactor);
        void recoveryFactor(double recoveryFactor);

    protected:
        double m_fatigueRate;
        double m_recoveryRate;
        double m_developFactor;
        double m_recoveryFactor;

};

#endif // S2MMUSCLEFATIGUEPARAM_H
