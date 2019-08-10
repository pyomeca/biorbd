#define BIORBD_API_EXPORTS
#include "s2mStaticOptimization.h"

#include <IpIpoptApplication.hpp>
#include "s2mMusculoSkeletalModel.h"
#include "Utils/Error.h"
#include "s2mMuscleStateDynamics.h"
#include "s2mStaticOptimizationIpoptLinearized.h"

s2mStaticOptimization::s2mStaticOptimization(s2mMusculoSkeletalModel& model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const s2mVector &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose
        ):
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_alreadyRun(false)
{
    m_allQ.push_back(Q);
    m_allQdot.push_back(Qdot);
    m_allTauTarget.push_back(tauTarget);

    if (initialActivationGuess.size() == 0){
        m_initialActivationGuess = s2mVector(m_model.nbMuscleTotal());
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            m_initialActivationGuess[i] = 0.01;
    }
    else
        m_initialActivationGuess = initialActivationGuess;
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel& model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const std::vector<s2mMuscleStateDynamics> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose
        ):
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_alreadyRun(false)
{
    m_allQ.push_back(Q);
    m_allQdot.push_back(Qdot);
    m_allTauTarget.push_back(tauTarget);

    m_initialActivationGuess = s2mVector(m_model.nbMuscleTotal());
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++)
        m_initialActivationGuess[i] = initialActivationGuess[i].activation();
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &model,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mTau> &allTauTarget,
        const s2mVector &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose) :
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allTauTarget(allTauTarget),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_alreadyRun(false)
{
    if (initialActivationGuess.size() == 0){
        m_initialActivationGuess = s2mVector(m_model.nbMuscleTotal());
        for (unsigned int i=0; i<m_model.nbMuscleTotal(); ++i)
            m_initialActivationGuess[i] = 0.01;
    }
    else
        m_initialActivationGuess = initialActivationGuess;
}

s2mStaticOptimization::s2mStaticOptimization(s2mMusculoSkeletalModel& model,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mTau> &allTauTarget,
        const std::vector<s2mMuscleStateDynamics> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose):
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allTauTarget(allTauTarget),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_alreadyRun(false)
{
    m_initialActivationGuess = s2mVector(m_model.nbMuscleTotal());
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++)
        m_initialActivationGuess[i] = initialActivationGuess[i].activation();
}

void s2mStaticOptimization::run(bool LinearizedState)
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
    s2mError::s2mAssert(status == Ipopt::Solve_Succeeded, "Ipopt initialization failed");

    for (unsigned int i=0; i<m_allQ.size(); ++i){
        if (LinearizedState)
            m_staticOptimProblem.push_back(
                        new s2mStaticOptimizationIpoptLinearized(
                            m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_initialActivationGuess,
                            m_useResidualTorque, m_pNormFactor, m_verbose
                            )
                        );
        else
            m_staticOptimProblem.push_back(
                        new s2mStaticOptimizationIpopt(
                            m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_initialActivationGuess,
                            m_useResidualTorque, m_pNormFactor, m_verbose
                            )
                        );
        // Optimize!
        status = app->OptimizeTNLP(m_staticOptimProblem[i]);

        // Take the solution of the previous optimization as the solution for the next optimization
        m_initialActivationGuess = static_cast<s2mStaticOptimizationIpopt*>(Ipopt::GetRawPtr(m_staticOptimProblem[i]))->finalSolution();
    }
    m_alreadyRun = true;
}

std::vector<s2mVector> s2mStaticOptimization::finalSolution()
{
    std::vector<s2mVector> res;
    if (!m_alreadyRun){
        s2mError::s2mAssert(0, "Problem has not been ran through the optimization process yet, you should optimize it first to get the optimized solution");
    }
    else {
        for (unsigned int i=0; i<m_allQ.size(); ++i){
            res.push_back(static_cast<s2mStaticOptimizationIpopt*>(Ipopt::GetRawPtr(m_staticOptimProblem[i]))->finalSolution());
        }
    }

    return res;
}

s2mVector s2mStaticOptimization::finalSolution(unsigned int index)
{
    s2mVector res;
    if (!m_alreadyRun){
        s2mError::s2mAssert(0, "Problem has not been ran through the optimization process yet, you should optimize it first to get the optimized solution");
    }
    else {
        res = static_cast<s2mStaticOptimizationIpopt*>(Ipopt::GetRawPtr(m_staticOptimProblem[index]))->finalSolution();
        }
    return res;
}

