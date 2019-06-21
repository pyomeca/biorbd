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
#include "s2mStaticOptimizationIpopt.h"


    
class BIORBD_API s2mStaticOptimization
{
    public:
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q_init,
                const s2mGenCoord& QDot_init,
                const s2mGenCoord& Tau,
                const s2mVector& Alpha_init,
                const int p = 2
                );
        int optimize();



    protected:
        s2mMusculoSkeletalModel m_model;
        s2mGenCoord m_Q;
        s2mGenCoord m_Qdot;
        s2mGenCoord m_Qddot;
        s2mVector m_Activ;
        unsigned int m_p;

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
