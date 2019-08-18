#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_H

#include <vector>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
class Model;

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace muscles {

class StateDynamics;
class BIORBD_API StaticOptimization
{
public:
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueTarget,
            const biorbd::utils::Vector& initialActivationGuess = biorbd::utils::Vector(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allGeneralizedTorqueTarget,
            const biorbd::utils::Vector& initialActivationGuess = biorbd::utils::Vector(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allGeneralizedTorqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);

    void run(bool useLinearizedState = false);
    std::vector<biorbd::utils::Vector> finalSolution();
    biorbd::utils::Vector finalSolution(unsigned int index);

protected:
    biorbd::Model& m_model;
    bool m_useResidualTorque;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> m_allQ;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> m_allQdot;
    std::vector<biorbd::rigidbody::GeneralizedTorque> m_allGeneralizedTorqueTarget;
    biorbd::utils::Vector m_initialActivationGuess;
    unsigned int m_pNormFactor;
    int m_verbose;
    std::vector<Ipopt::SmartPtr<Ipopt::TNLP>> m_staticOptimProblem;
    bool m_alreadyRun;

};

}}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
