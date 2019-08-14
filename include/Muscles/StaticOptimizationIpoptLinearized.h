#ifndef S2M_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
#define S2M_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H

#include "biorbdConfig.h"
#include "Utils/Matrix.h"
#include "Muscles/StaticOptimizationIpopt.h"

class BIORBD_API s2mStaticOptimizationIpoptLinearized : public s2mStaticOptimizationIpopt
{
public:
    s2mStaticOptimizationIpoptLinearized(
            s2mMusculoSkeletalModel& model,
            const biorbd::utils::GenCoord& Q,
            const biorbd::utils::GenCoord& Qdot,
            const biorbd::utils::Tau& tauTarget,
            const biorbd::utils::Vector& activationInit,
            bool useResidual = true,
            unsigned int pNormFactor = 2,
            int verbose = 0,
            double eps = 1e-10);

    virtual ~s2mStaticOptimizationIpoptLinearized();

    // Method to return the constraint residuals
    virtual bool eval_g(
       Ipopt::Index n,
       const Ipopt::Number* x,
       bool new_x,
       Ipopt::Index m,
       Ipopt::Number* g);

    // Method to return:
    //   1) The structure of the jacobian (if "values" is NULL)
    //   2) The values of the jacobian (if "values" is not NULL)
    virtual bool eval_jac_g(
       Ipopt::Index n,
       const Ipopt::Number* x,
       bool new_x,
       Ipopt::Index m,
       Ipopt::Index nele_jac,
       Ipopt::Index* iRow,
       Ipopt::Index* jCol,
       Ipopt::Number* values);

protected:
    biorbd::utils::Matrix m_jacobian;
    void prepareJacobian();

};

#endif // S2M_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
