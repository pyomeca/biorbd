#define BIORBD_API_EXPORTS
#include "Muscles/StaticOptimization.h"

#include <IpIpoptApplication.hpp>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Vector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/StaticOptimizationIpoptLinearized.h"

biorbd::muscles::StaticOptimization::StaticOptimization(biorbd::Model &model) :
    m_model(model),
    m_useResidualTorque(std::make_shared<bool>()),
    m_allQ(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allQdot(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allGeneralizedTorqueTarget(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedTorque>>()),
    m_initialActivationGuess(std::make_shared<biorbd::utils::Vector>()),
    m_pNormFactor(std::make_shared<unsigned int>()),
    m_verbose(std::make_shared<int>()),
    m_staticOptimProblem(std::make_shared<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>>()),
    m_alreadyRun(std::make_shared<bool>(false))
{

}

biorbd::muscles::StaticOptimization::StaticOptimization(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedTorque &GeneralizedTorqueTarget,
        const biorbd::utils::Vector &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose) :
    m_model(model),
    m_useResidualTorque(std::make_shared<bool>(useResidualTorque)),
    m_allQ(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allQdot(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allGeneralizedTorqueTarget(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedTorque>>()),
    m_initialActivationGuess(std::make_shared<biorbd::utils::Vector>()),
    m_pNormFactor(std::make_shared<unsigned int>(pNormFactor)),
    m_verbose(std::make_shared<int>(verbose)),
    m_staticOptimProblem(std::make_shared<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>>()),
    m_alreadyRun(std::make_shared<bool>(false))
{
    m_allQ->push_back(Q);
    m_allQdot->push_back(Qdot);
    m_allGeneralizedTorqueTarget->push_back(GeneralizedTorqueTarget);

    if (initialActivationGuess.size() == 0){
        *m_initialActivationGuess = biorbd::utils::Vector(m_model.nbMuscleTotal());
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            (*m_initialActivationGuess)[i] = 0.01;
    }
    else
        *m_initialActivationGuess = initialActivationGuess;
}

biorbd::muscles::StaticOptimization::StaticOptimization(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedTorque &GeneralizedTorqueTarget,
        const std::vector<biorbd::muscles::StateDynamics> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose
        ) :
    m_model(model),
    m_useResidualTorque(std::make_shared<bool>(useResidualTorque)),
    m_allQ(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allQdot(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>()),
    m_allGeneralizedTorqueTarget(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedTorque>>()),
    m_initialActivationGuess(std::make_shared<biorbd::utils::Vector>()),
    m_pNormFactor(std::make_shared<unsigned int>(pNormFactor)),
    m_verbose(std::make_shared<int>(verbose)),
    m_staticOptimProblem(std::make_shared<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>>()),
    m_alreadyRun(std::make_shared<bool>(false))
{
    m_allQ->push_back(Q);
    m_allQdot->push_back(Qdot);
    m_allGeneralizedTorqueTarget->push_back(GeneralizedTorqueTarget);

    *m_initialActivationGuess = biorbd::utils::Vector(m_model.nbMuscleTotal());
    if (initialActivationGuess.size() == 0){
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            (*m_initialActivationGuess)[i] = 0.01;
    } else {
        for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++)
            (*m_initialActivationGuess)[i] = initialActivationGuess[i].activation();
    }
}

biorbd::muscles::StaticOptimization::StaticOptimization(
        biorbd::Model &model,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates> &allQ,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates> &allQdot,
        const std::vector<biorbd::rigidbody::GeneralizedTorque> &allGeneralizedTorqueTarget,
        const biorbd::utils::Vector &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose) :
    m_model(model),
    m_useResidualTorque(std::make_shared<bool>(useResidualTorque)),
    m_allQ(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>(allQ)),
    m_allQdot(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>(allQdot)),
    m_allGeneralizedTorqueTarget(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedTorque>>(allGeneralizedTorqueTarget)),
    m_initialActivationGuess(std::make_shared<biorbd::utils::Vector>()),
    m_pNormFactor(std::make_shared<unsigned int>(pNormFactor)),
    m_verbose(std::make_shared<int>(verbose)),
    m_staticOptimProblem(std::make_shared<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>>()),
    m_alreadyRun(std::make_shared<bool>(false))
{
    if (initialActivationGuess.size() == 0){
        *m_initialActivationGuess = biorbd::utils::Vector(m_model.nbMuscleTotal());
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            (*m_initialActivationGuess)[i] = 0.01;
    }
    else
        *m_initialActivationGuess = initialActivationGuess;
}

biorbd::muscles::StaticOptimization::StaticOptimization(
        biorbd::Model& model,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates> &allQ,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates> &allQdot,
        const std::vector<biorbd::rigidbody::GeneralizedTorque> &allGeneralizedTorqueTarget,
        const std::vector<biorbd::muscles::StateDynamics> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose):
    m_model(model),
    m_useResidualTorque(std::make_shared<bool>(useResidualTorque)),
    m_allQ(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>(allQ)),
    m_allQdot(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedCoordinates>>(allQdot)),
    m_allGeneralizedTorqueTarget(std::make_shared<std::vector<biorbd::rigidbody::GeneralizedTorque>>(allGeneralizedTorqueTarget)),
    m_initialActivationGuess(std::make_shared<biorbd::utils::Vector>()),
    m_pNormFactor(std::make_shared<unsigned int>(pNormFactor)),
    m_verbose(std::make_shared<int>(verbose)),
    m_staticOptimProblem(std::make_shared<std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>>()),
    m_alreadyRun(std::make_shared<bool>(false))
{
    *m_initialActivationGuess = biorbd::utils::Vector(m_model.nbMuscleTotal());
    if (initialActivationGuess.size() == 0){
        *m_initialActivationGuess = biorbd::utils::Vector(m_model.nbMuscleTotal());
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            (*m_initialActivationGuess)[i] = 0.01;
    } else {
        for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++)
            (*m_initialActivationGuess)[i] = initialActivationGuess[i].activation();
    }

}

biorbd::muscles::StaticOptimization biorbd::muscles::StaticOptimization::DeepCopy() const
{
    biorbd::muscles::StaticOptimization copy(this->m_model);
    *copy.m_useResidualTorque = *m_useResidualTorque;
    copy.m_allQ->resize(m_allQ->size());
    for (unsigned int i=0; i<m_allQ->size(); ++i)
        (*copy.m_allQ)[i] = (*m_allQ)[i].DeepCopy();
    copy.m_allQdot->resize(m_allQdot->size());
    for (unsigned int i=0; i<m_allQdot->size(); ++i)
        (*copy.m_allQdot)[i] = (*m_allQdot)[i].DeepCopy();
    copy.m_allGeneralizedTorqueTarget->resize(m_allGeneralizedTorqueTarget->size());
    for (unsigned int i=0; i<m_allGeneralizedTorqueTarget->size(); ++i)
        (*copy.m_allGeneralizedTorqueTarget)[i] = (*m_allGeneralizedTorqueTarget)[i].DeepCopy();
    *copy.m_initialActivationGuess = m_initialActivationGuess->DeepCopy();
    *copy.m_pNormFactor = *m_pNormFactor;
    *copy.m_verbose = *m_verbose;
    *copy.m_staticOptimProblem = *m_staticOptimProblem;
    *copy.m_alreadyRun = *m_alreadyRun;
    return copy;
}

void biorbd::muscles::StaticOptimization::run(bool LinearizedState)
{
    // Setup the Ipopt problem
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    //app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("derivative_test", "first-order");
    app->Options()->SetIntegerValue("max_iter", 10000);

    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    biorbd::utils::Error::error(status == Ipopt::Solve_Succeeded, "Ipopt initialization failed");

    for (unsigned int i=0; i<m_allQ->size(); ++i){
        if (LinearizedState)
            m_staticOptimProblem->push_back(
                        new biorbd::muscles::StaticOptimizationIpoptLinearized(
                            m_model, (*m_allQ)[i], (*m_allQdot)[i], (*m_allGeneralizedTorqueTarget)[i], *m_initialActivationGuess,
                            *m_useResidualTorque, *m_pNormFactor, *m_verbose
                            )
                        );
        else
            m_staticOptimProblem->push_back(
                        new biorbd::muscles::StaticOptimizationIpopt(
                            m_model, (*m_allQ)[i], (*m_allQdot)[i], (*m_allGeneralizedTorqueTarget)[i], *m_initialActivationGuess,
                            *m_useResidualTorque, *m_pNormFactor, *m_verbose
                            )
                        );
        // Optimize!
        status = app->OptimizeTNLP((*m_staticOptimProblem)[i]);

        // Take the solution of the previous optimization as the solution for the next optimization
        *m_initialActivationGuess = static_cast<biorbd::muscles::StaticOptimizationIpopt*>(Ipopt::GetRawPtr((*m_staticOptimProblem)[i]))->finalSolution();
    }
    *m_alreadyRun = true;
}

std::vector<biorbd::utils::Vector> biorbd::muscles::StaticOptimization::finalSolution()
{
    std::vector<biorbd::utils::Vector> res;
    if (!*m_alreadyRun){
        biorbd::utils::Error::error(0, "Problem has not been ran through the optimization process yet, you should optimize it first to get the optimized solution");
    }
    else {
        for (unsigned int i=0; i<m_allQ->size(); ++i){
            res.push_back(static_cast<biorbd::muscles::StaticOptimizationIpopt*>(Ipopt::GetRawPtr((*m_staticOptimProblem)[i]))->finalSolution());
        }
    }

    return res;
}

biorbd::utils::Vector biorbd::muscles::StaticOptimization::finalSolution(unsigned int index)
{
    biorbd::utils::Vector res;
    if (!*m_alreadyRun){
        biorbd::utils::Error::error(0, "Problem has not been ran through the optimization process yet, you should optimize it first to get the optimized solution");
    }
    else {
        res = static_cast<biorbd::muscles::StaticOptimizationIpopt*>(Ipopt::GetRawPtr((*m_staticOptimProblem)[index]))->finalSolution();
        }
    return res;
}

