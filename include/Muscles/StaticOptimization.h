#ifndef S2M_STATIC_OPTIMIZATION_H
#define S2M_STATIC_OPTIMIZATION_H

#include <vector>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"
#include "Utils/Vector.h"

class s2mMusculoSkeletalModel;
namespace biorbd { namespace utils {
class GenCoord;
class Tau;
}}
class s2mMuscleStateDynamics;
class BIORBD_API s2mStaticOptimization
{
public:
    s2mStaticOptimization(
            s2mMusculoSkeletalModel& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& Qdot,
            const biorbd::utils::Tau& tauTarget,
            const biorbd::utils::Vector& initialActivationGuess = biorbd::utils::Vector(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    s2mStaticOptimization(
            s2mMusculoSkeletalModel& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& Qdot,
            const biorbd::utils::Tau& tauTarget,
            const std::vector<s2mMuscleStateDynamics>& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    s2mStaticOptimization(
            s2mMusculoSkeletalModel& model,
            const std::vector<biorbd::utils::GenCoord>& allQ,
            const std::vector<biorbd::utils::GenCoord>& allQdot,
            const std::vector<biorbd::utils::Tau>& allTauTarget,
            const biorbd::utils::Vector& initialActivationGuess = biorbd::utils::Vector(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    s2mStaticOptimization(
            s2mMusculoSkeletalModel& model,
            const std::vector<biorbd::utils::GenCoord>& allQ,
            const std::vector<biorbd::utils::GenCoord>& allQdot,
            const std::vector<biorbd::utils::Tau>& allTauTarget,
            const std::vector<s2mMuscleStateDynamics>& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);

    void run(bool useLinearizedState = false);
    std::vector<biorbd::utils::Vector> finalSolution();
    biorbd::utils::Vector finalSolution(unsigned int index);

protected:
    s2mMusculoSkeletalModel& m_model;
    bool m_useResidualTorque;
    std::vector<biorbd::utils::GenCoord> m_allQ;
    std::vector<biorbd::utils::GenCoord> m_allQdot;
    std::vector<biorbd::utils::Tau> m_allTauTarget;
    biorbd::utils::Vector m_initialActivationGuess;
    unsigned int m_pNormFactor;
    int m_verbose;
    std::vector<Ipopt::SmartPtr<Ipopt::TNLP>> m_staticOptimProblem;
    bool m_alreadyRun;

};
#endif // S2M_STATIC_OPTIMIZATION_H
