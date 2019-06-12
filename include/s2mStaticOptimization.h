#ifndef S2MSTATICOPTIMIZATION_H
#define S2MSTATICOPTIMIZATION_H

#include "biorbdConfig.h"
#include "s2mGenCoord.h"

    
class BIORBD_API s2mStaticOptimization
{
    public:
        s2mStaticOptimization(
                const s2mGenCoord Q_init,
                const s2mGenCoord QDot_init,
                const s2mGenCoord Tau,
                const s2mGenCoord Alpha_init,
                const int p = 2
                );

        //virtual void run();

    protected:

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
