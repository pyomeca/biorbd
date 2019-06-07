#ifndef S2MMUSCLEFATIGUEPARAM_H
#define S2MMUSCLEFATIGUEPARAM_H
    #include "biorbdConfig.h"

class BIORBD_API s2mMuscleFatigueParam
{
    public:
        s2mMuscleFatigueParam(double param1 = 0);

        // Get and Set
        double param1() const;
        void param1(double param1);

    protected:
        double m_param1;

    private:

};

#endif // S2MMUSCLEFATIGUEPARAM_H
