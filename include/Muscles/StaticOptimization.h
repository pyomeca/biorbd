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
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace muscles {
class StateDynamics;

///
/// \brief Base class for a Static Optimization algorithm
///
class BIORBD_API StaticOptimization
{
public:
    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    ///
    StaticOptimization(
            biorbd::Model& model);
            
    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedVelocity& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
            const biorbd::utils::Vector& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);

    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedVelocity& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess = std::vector<biorbd::muscles::StateDynamics>(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);

    ///
    /// \brief Construct static optimization for multiple frames
    /// \param model The musculoskeletal Model
    /// \param allQ The generalized coordinates
    /// \param allQdot The generalized velocities
    /// \param allTorqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedVelocity>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allTorqueTarget,
            const biorbd::utils::Vector& initialActivationGuess,
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);

    ///
    /// \brief Construct static optimization for multiple frames
    /// \param model The musculoskeletal Model
    /// \param allQ The generalized coordinates
    /// \param allQdot The generalized velocities
    /// \param allTorqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
            biorbd::Model& model,
            const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
            const std::vector<biorbd::rigidbody::GeneralizedVelocity>& allQdot,
            const std::vector<biorbd::rigidbody::GeneralizedTorque>& allTorqueTarget,
            const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess = std::vector<biorbd::muscles::StateDynamics>(),
            unsigned int pNormFactor = 2,
            bool useResidualTorque = true,
            int verbose = 0);
            
    ///
    /// \brief Deep copy of the static optimization
    /// \return A deep copy of the static optimization
    ///
    biorbd::muscles::StaticOptimization DeepCopy() const;

    ///
    /// \brief Run the static optimization
    /// \param useLinearizedState If use the algorithm should be run with the linearized approach (faster but less precise)
    ///
    void run(bool useLinearizedState = true);
    
    ///
    /// \brief Return the final solution
    /// \return The final solution
    ///
    std::vector<biorbd::utils::Vector> finalSolution();
    
    ///
    /// \brief Return the final solution at a specific index
    /// \return The final solution at a specific index
    ///
    biorbd::utils::Vector finalSolution(unsigned int index);

protected:
    biorbd::Model& m_model; ///< A reference to the model
    std::shared_ptr<bool> m_useResidualTorque; ///< To use residual torque
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedCoordinates>> m_allQ; ///< All the generalized coordinates
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedVelocity>> m_allQdot; ///< All the generalized velocities
    std::shared_ptr<std::vector<biorbd::rigidbody::GeneralizedTorque>> m_allTorqueTarget; ///< All the torque targets
    std::shared_ptr<biorbd::utils::Vector> m_initialActivationGuess; ///< Initial activation guess 
    std::shared_ptr<unsigned int> m_pNormFactor; ///< The p-norm factor
    std::shared_ptr<int> m_verbose; ///<Verbose level
    std::shared_ptr<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>> m_staticOptimProblem; ///<The static optimization problem
    std::shared_ptr<bool> m_alreadyRun; ///< If already ran the static optimization

};

}}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
