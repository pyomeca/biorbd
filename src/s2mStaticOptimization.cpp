#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimization.h"


s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mGenCoord& Qddot,
        const s2mVector& Activ,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_Tau(s2mTau(m)),
    m_Activ(Activ),
    m_p(p)
{
    m_Tau.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_Tau);
    std::cout << "m_Tau\n:" << m_Tau << std::endl;
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &Tau,
        const s2mVector &Activ,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Tau(Tau),
    m_Activ(Activ),
    m_p(p)
{

}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mGenCoord &Qddot,
        const std::vector<s2mMuscleStateActual> &State,
        const unsigned int p
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_Tau(s2mTau(m)),
    m_State(State),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_p(p)
{
    m_Tau.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_Tau);
    std::cout << "m_Tau\n:" << m_Tau << std::endl;
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_State[i].activation();
    }
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &Tau,
        const std::vector<s2mMuscleStateActual> &State,
        const unsigned int p):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Tau(Tau),
    m_State(State),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_p(p)
{
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_State[i].activation();
    }
}

int s2mStaticOptimization::optimize(
        bool LinearizedState
        )
{
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp;
    if (LinearizedState){
        std::cout << "Linearized" << std::endl;
        mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_Q, m_Qdot, m_Tau, m_Activ);
    }
    else {
        mynlp = new s2mStaticOptimizationIpopt(m_model, m_Q, m_Qdot, m_Tau, m_Activ);
        std::cout << "m_Tau\n:" << m_Tau << std::endl;
    }
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetStringValue("derivative_test", "first-order");
    Ipopt::ApplicationReturnStatus status;
   status = app->Initialize();
   if( status != Ipopt::Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return status;
   }

   // Ask Ipopt to solve the problem
   status = app->OptimizeTNLP(mynlp);

   if( status == Ipopt::Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
   }
   else
   {
      std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
   }
   return status;

}

