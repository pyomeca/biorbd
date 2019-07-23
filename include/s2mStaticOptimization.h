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
                s2mMusculoSkeletalModel& model,
                const s2mGenCoord& Q,
                const s2mGenCoord& Qdot,
                const s2mTau& tauTarget,
                const s2mVector& initialActivationGuess = s2mVector(),
                unsigned int pNormFactor = 2,
                bool useResidualTorque = true,
                int verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel& model,
                const s2mGenCoord& Q,
                const s2mGenCoord& Qdot,
                const s2mTau& tauTarget,
                const std::vector<s2mMuscleStateActual>& initialActivationGuess,
                unsigned int pNormFactor = 2,
                bool useResidualTorque = true,
                int verbose = 0
                );

        s2mStaticOptimization(
                s2mMusculoSkeletalModel& model,
                const std::vector<s2mGenCoord>& allQ,
                const std::vector<s2mGenCoord>& allQdot,
                const std::vector<s2mTau>& allTauTarget,
                const s2mVector& initialActivationGuess = s2mVector(),
                unsigned int pNormFactor = 2,
                bool useResidualTorque = true,
                int verbose = 0
                );
        s2mStaticOptimization(
                s2mMusculoSkeletalModel& model,
                const std::vector<s2mGenCoord>& allQ,
                const std::vector<s2mGenCoord>& allQdot,
                const std::vector<s2mTau>& allTauTarget,
                const std::vector<s2mMuscleStateActual>& initialActivationGuess,
                unsigned int pNormFactor = 2,
                bool useResidualTorque = true,
                int verbose = 0
                );

        void run(bool useLinearizedState = false);

    protected:
        s2mMusculoSkeletalModel& m_model;
        bool m_useResidualTorque;
        std::vector<s2mGenCoord> m_allQ;
        std::vector<s2mGenCoord> m_allQdot;
        std::vector<s2mTau> m_allTauTarget;
        s2mVector m_initialActivationGuess;
        unsigned int m_pNormFactor;
        int m_verbose;
        std::vector<Ipopt::SmartPtr<s2mStaticOptimizationIpopt>> m_staticOptimProblem;

    private:

};
#endif // S2MSTATICOPTIMIZATION_H
