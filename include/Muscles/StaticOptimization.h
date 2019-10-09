#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_H

#include <vector>
#include <memory>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"

namespace biorbd {
class Model;

namespace utils {
class Vector;
}

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
            biorbd::Model& model);
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueTarget,
            const biorbd::utils::Vector& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess = std::vector<biorbd::muscles::StateDynamics>(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allGeneralizedTorqueTarget,
            const biorbd::utils::Vector& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allGeneralizedTorqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess = std::vector<biorbd::muscles::StateDynamics>(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
    biorbd::muscles::StaticOptimization DeepCopy() const;

    void run(bool useLinearizedState = false);
    std::vector<biorbd::utils::Vector> finalSolution();
    biorbd::utils::Vector finalSolution(unsigned int index);

protected:
    biorbd::Model& m_model;
    std::shared_ptr<bool> m_useResidualTorque;
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedCoordinates>> m_allQ;
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedCoordinates>> m_allQdot;
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedTorque>> m_allGeneralizedTorqueTarget;
    std::shared_ptr<biorbd::utils::Vector> m_initialActivationGuess;
    std::shared_ptr<unsigned int> m_pNormFactor;
    std::shared_ptr<int> m_verbose;
    std::shared_ptr<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>> m_staticOptimProblem;
    std::shared_ptr<bool> m_alreadyRun;

};

}}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
