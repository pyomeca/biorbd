#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimization.h"


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
    m_verbose(verbose)
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
        const std::vector<s2mMuscleStateActual> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose
        ):
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose)
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
    m_verbose(verbose)
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
        const std::vector<s2mMuscleStateActual> &initialActivationGuess,
        unsigned int pNormFactor,
        bool useResidualTorque,
        int verbose):
    m_model(model),
    m_useResidualTorque(useResidualTorque),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allTauTarget(allTauTarget),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose)
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

        status = app->OptimizeTNLP(static_cast<Ipopt::SmartPtr<Ipopt::TNLP>>(*m_staticOptimProblem.end()));

        // Take the solution of the previous optimization as the solution for the next optimization
        m_initialActivationGuess = static_cast<s2mStaticOptimizationIpopt*>(Ipopt::GetRawPtr(*m_staticOptimProblem.end()))->finalSolution();
    }
}

