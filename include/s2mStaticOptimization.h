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
#include "s2mStaticOptimizationIpOptLinearized.h"


    
class BIORBD_API s2mStaticOptimization
{
    public:
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mGenCoord& Qddot,
                const s2mVector& Activ,
                const unsigned int p = 2
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mTau& Tau,
                const s2mVector& Activ,
                const unsigned int p = 2
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mGenCoord& Qddot,
                const std::vector<s2mMuscleStateActual>& Activ,
                const unsigned int p = 2
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mTau& Tau,
                const std::vector<s2mMuscleStateActual>& Activ,
                const unsigned int p = 2
                );

        int optimize(
                bool LinearizedState = false
                );



    protected:
        s2mMusculoSkeletalModel m_model;
        s2mGenCoord m_Q;
        s2mGenCoord m_Qdot;
        s2mGenCoord m_Qddot;
        s2mTau m_Tau;
        std::vector<s2mMuscleStateActual> m_State;
        s2mVector m_Activ;
        unsigned int m_p;

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
