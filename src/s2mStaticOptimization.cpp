#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimization.h"


s2mStaticOptimization::s2mStaticOptimization(
        s2mMusculoSkeletalModel m,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mGenCoord& Tau,
        const s2mVector& Activ,
        const int p
        )
{

}

int s2mStaticOptimization::run(s2mMusculoSkeletalModel m, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const s2mGenCoord &Tau, const s2mVector &Activ, const int p)
{
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new s2mStaticOptimizationIpopt(m, Q, Qdot, Tau, Activ);
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
      return (int) status;
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
   return (int) status;

}

