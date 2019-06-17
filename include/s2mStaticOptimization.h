#ifndef S2MSTATICOPTIMIZATION_H
#define S2MSTATICOPTIMIZATION_H

#include "biorbdConfig.h"
#include "s2mGenCoord.h"
#include "s2mMusculoSkeletalModel.h"
#include "s2mMuscles.h"
#include "s2mVector.h"
#include <Eigen/Dense>
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include "s2mStaticOptimizationIpOpt.h"


    
class BIORBD_API s2mStaticOptimization
{
    public:
        s2mStaticOptimization(
                s2mMusculoSkeletalModel m,
                const s2mGenCoord& Q_init,
                const s2mGenCoord& QDot_init,
                const s2mGenCoord& Tau,
                const s2mGenCoord& Alpha_init,
                const int p = 2
                );
        int run(
                int   argv,
                char* argc[],
                s2mMusculoSkeletalModel m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mGenCoord& Tau,
                const s2mGenCoord& Activ,
                const int p
               );



    protected:

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
