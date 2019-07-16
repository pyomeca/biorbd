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
                double recoverFactor = 0);

        // Get and Set
        double fatigueRate() const;
        double recoveryRate() const;
        double developFactor() const;
        double recoverFactor() const;

        void fatigueRate(double fatigueRate);
        void recoveryRate(double recoveryRate);
        void developFactor(double developFactor);
        void recoverFactor(double recoverFactor);

    protected:
        double m_fatigueRate;
        double m_recoveryRate;
        double m_developFactor;
        double m_recoverFactor;

};

#endif // S2MMUSCLEFATIGUEPARAM_H
