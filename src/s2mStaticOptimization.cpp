#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimization.h"


s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mGenCoord& Qddot,
        const s2mVector& Activ,
        const unsigned int pNormFactor,
        const int verbose
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_tauTarget(s2mTau(m)),
    m_Activ(Activ),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(1)
{
    std::vector<s2mGenCoord> allQ;
    allQ.push_back(Q);
    m_tauTarget.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_tauTarget);
    std::cout << "m_Tau\n:" << m_tauTarget << std::endl;
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const s2mVector &Activ,
        const unsigned int pNormFactor,
        const int verbose
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_tauTarget(tauTarget),
    m_Activ(Activ),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(1)
{
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mGenCoord &Qddot,
        const std::vector<s2mMuscleStateActual> &state,
        const unsigned int pNormFactor,
        const int verbose
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_Qddot(Qddot),
    m_tauTarget(s2mTau(m)),
    m_state(state),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(1)
{
    m_tauTarget.setZero();
    RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, m_tauTarget);
    std::cout << "m_Tau\n:" << m_tauTarget << std::endl;
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_state[i].activation();
    }
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const std::vector<s2mMuscleStateActual> &state,
        const unsigned int pNormFactor,
        const int verbose
        ):
    m_model(m),
    m_Q(Q),
    m_Qdot(Qdot),
    m_tauTarget(tauTarget),
    m_state(state),
    m_Activ(s2mVector(m.nbMuscleTotal())),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(1)
{
    m_Activ.setZero();
    for (unsigned int i = 0; i<m_model.nbMuscleTotal(); i++){
        m_Activ[i] = m_state[i].activation();
    }
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}


//constructors for a vector of instants to be optimized by ipopt
s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mGenCoord> &allQddot,
        const std::vector<s2mVector> &allActiv,
        const unsigned int pNormFactor,
        const int verbose):
    m_model(m),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allQddot(allQddot),
    m_allActiv(allActiv),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(static_cast<unsigned int>(allQ.size())),
    m_configConstructor(1)

{
    unsigned int nbInstant = static_cast<unsigned int>(allQ.size());
    if (nbInstant != allQdot.size())
        s2mError::s2mAssert(false, "allQdot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allQddot.size())
        s2mError::s2mAssert(false, "allQddot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allActiv.size())
        s2mError::s2mAssert(false, "allActiv is not the same size as allQ, numbers of instant to be optimized must be equal");
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mTau> &allTauTarget,
        const std::vector<s2mVector> &allActiv,
        const unsigned int pNormFactor,
        const int verbose):
    m_model(m),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allTauTarget(allTauTarget),
    m_allActiv(allActiv),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(static_cast<unsigned int>(allQ.size())),
    m_configConstructor(2)
{
    unsigned int nbInstant = static_cast<unsigned int>(allQ.size());
    if (nbInstant != allQdot.size())
        s2mError::s2mAssert(false, "allQdot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allTauTarget.size())
        s2mError::s2mAssert(false, "allQddot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allActiv.size())
        s2mError::s2mAssert(false, "allActiv is not the same size as allQ, numbers of instant to be optimized must be equal");
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mGenCoord> &allQddot,
        const std::vector<std::vector<s2mMuscleStateActual> > &allState,
        const unsigned int pNormFactor,
        const int verbose):
    m_model(m),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allQddot(allQddot),
    m_allState(allState),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(static_cast<unsigned int>(allQ.size())),
    m_configConstructor(3)
{
    unsigned int nbInstant = static_cast<unsigned int>(allQ.size());
    if (nbInstant != allQdot.size())
        s2mError::s2mAssert(false, "allQdot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allQddot.size())
        s2mError::s2mAssert(false, "allQddot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allState.size())
        s2mError::s2mAssert(false, "allActiv is not the same size as allQ, numbers of instant to be optimized must be equal");
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;}
}

s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel &m,
        const std::vector<s2mGenCoord> &allQ,
        const std::vector<s2mGenCoord> &allQdot,
        const std::vector<s2mTau> &allTauTarget,
        const std::vector<std::vector<s2mMuscleStateActual> > &allState,
        const unsigned int pNormFactor,
        const int verbose):
    m_model(m),
    m_allQ(allQ),
    m_allQdot(allQdot),
    m_allTauTarget(allTauTarget),
    m_allState(allState),
    m_pNormFactor(pNormFactor),
    m_verbose(verbose),
    m_finalSolution(std::vector<s2mVector>(m.nbMuscleTotal())),
    m_multipleInstant(static_cast<unsigned int>(allQ.size())),
    m_configConstructor(4)
{
    unsigned int nbInstant = static_cast<unsigned int>(allQ.size());
    if (nbInstant != allQdot.size())
        s2mError::s2mAssert(false, "allQdot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allTauTarget.size())
        s2mError::s2mAssert(false, "allQddot is not the same size as allQ, numbers of instant to be optimized must be equal");
    if (nbInstant != allState.size())
        s2mError::s2mAssert(false, "allActiv is not the same size as allQ, numbers of instant to be optimized must be equal");
    for (unsigned int i=0;i<m_model.nbMuscleTotal();i++) {
        s2mVector x(m_multipleInstant);
        m_finalSolution[i] = x;
    }
}


void s2mStaticOptimization::run(
        bool LinearizedState
        )
{
   Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
   app->Options()->SetNumericValue("tol", 1e-7);
   app->Options()->SetStringValue("mu_strategy", "adaptive");
   //app->Options()->SetStringValue("output_file", "ipopt.out");
   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
   app->Options()->SetStringValue("derivative_test", "first-order");
   app->Options()->SetIntegerValue("max_iter", 10000);
   Ipopt::ApplicationReturnStatus status;
   status = app->Initialize();
   if( status != Ipopt::Solve_Succeeded )
       s2mError::s2mAssert(false, "Ipopt initialization failed");
   if (m_multipleInstant == 1){
       Ipopt::SmartPtr<Ipopt::TNLP> mynlp;
       if (LinearizedState)
           mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_Q, m_Qdot, m_tauTarget, m_Activ, m_verbose, m_pNormFactor);
       else
           mynlp = new s2mStaticOptimizationIpopt(m_model, m_Q, m_Qdot, m_tauTarget, m_Activ, true, m_verbose, m_pNormFactor);
       status = app->OptimizeTNLP(mynlp);
       s2mVector solution(static_cast<Ipopt::SmartPtr<s2mStaticOptimizationIpopt>>(mynlp)->finalSolution());
       for (unsigned int j=0; j<m_model.nbMuscleTotal(); j++){
           m_finalSolution[j][0] = solution[j];
       }
   }
   else {
       for (unsigned int i=0; i<m_multipleInstant; ++i){
           Ipopt::SmartPtr<Ipopt::TNLP> mynlp;
           if (m_configConstructor == 1){
               if (LinearizedState){
                   mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_allQ[i], m_allQdot[i], m_allQddot[i], m_allActiv[i], m_pNormFactor, m_verbose);
               }
               else{
                   mynlp = new s2mStaticOptimizationIpopt(m_model, m_allQ[i], m_allQdot[i], m_allQddot[i], m_allActiv[i], m_pNormFactor, m_verbose);
               }
           }
           else if (m_configConstructor == 2){
               if (LinearizedState){
                   mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_allActiv[i], m_pNormFactor, m_verbose);
               }
               else{
                   mynlp = new s2mStaticOptimizationIpopt(m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_allActiv[i], m_pNormFactor, m_verbose);
               }
           }
           else if (m_configConstructor == 3){
               if (LinearizedState){
                   //mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_allQ[i], m_allQdot[i], m_allQddot[i], m_allState[i], m_pNormFactor, m_verbose);
               }
               else{

                   //mynlp = new s2mStaticOptimizationIpopt(m_model, m_allQ[i], m_allQdot[i], m_allQddot[i], m_allState[i], m_pNormFactor, m_verbose);
               }
           }
           else if (m_configConstructor == 4){
               if (LinearizedState){
                   //mynlp = new s2mStaticOptimizationIpoptLinearized(m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_allState[i], m_pNormFactor, m_verbose);
               }
               else{
                   //mynlp = new s2mStaticOptimizationIpopt(m_model, m_allQ[i], m_allQdot[i], m_allTauTarget[i], m_allState.operator[](i), m_pNormFactor, m_verbose);
               }
           }
           // Ask Ipopt to solve the problem
           status = app->OptimizeTNLP(mynlp);
           s2mVector solution(static_cast<Ipopt::SmartPtr<s2mStaticOptimizationIpopt>>(mynlp)->finalSolution());
           for (unsigned int j=0; j<m_model.nbMuscleTotal(); j++){
               m_finalSolution[j][i] = solution[j];
           }

        }
    }
}

