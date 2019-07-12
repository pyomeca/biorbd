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
#include "s2mStaticOptimizationIpoptLinearized.h"


    
class BIORBD_API s2mStaticOptimization
{
    public:
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mGenCoord& Qddot,
                const s2mVector& Activ,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mTau& tauTarget,
                const s2mVector& Activ,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mGenCoord& Qddot,
                const std::vector<s2mMuscleStateActual>& Activ,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const s2mGenCoord& Q, // states
                const s2mGenCoord& Qdot, // derived states
                const s2mTau& tauTarget,
                const std::vector<s2mMuscleStateActual>& Activ,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );

        //constructors for  a vector of instants to be optimized by ipopt
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const std::vector<s2mGenCoord>& allQ, // states
                const std::vector<s2mGenCoord>& allQdot, // derived states
                const std::vector<s2mGenCoord>& allQddot,
                const std::vector<s2mVector>& allActiv,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const std::vector<s2mGenCoord>& allQ,
                const std::vector<s2mGenCoord>& allQdot,
                const std::vector<s2mTau>& allTauTarget,
                const std::vector<s2mVector>& allActiv,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const std::vector<s2mGenCoord>& allQ,
                const std::vector<s2mGenCoord>& allQdot,
                const std::vector<s2mGenCoord>& allQddot,
                const std::vector<std::vector<s2mMuscleStateActual>>& allState,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel &m,
                const std::vector<s2mGenCoord>& allQ,
                const std::vector<s2mGenCoord>& allQdot,
                const std::vector<s2mTau>& allTauTarget,
                const std::vector<std::vector<s2mMuscleStateActual>>& allState,
                const unsigned int pNormFactor = 2,
                const bool verbose = 0
                );

        void run(
                bool LinearizedState = false
                );



    protected:
        s2mMusculoSkeletalModel m_model;
        s2mGenCoord m_Q;
        s2mGenCoord m_Qdot;
        s2mGenCoord m_Qddot;
        s2mTau m_tauTarget;
        std::vector<s2mMuscleStateActual> m_state;
        s2mVector m_Activ;
        unsigned int m_pNormFactor;
        bool m_verbose;

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
