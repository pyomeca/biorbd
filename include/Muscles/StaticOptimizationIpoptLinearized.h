#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H

#include "biorbdConfig.h"
#include "Muscles/StaticOptimizationIpopt.h"

namespace biorbd {
namespace utils {
class Matrix;
}

namespace muscles {
///
/// \brief Class StaticOptimizationIpoptLinearized
///
class BIORBD_API StaticOptimizationIpoptLinearized : public biorbd::muscles::StaticOptimizationIpopt
{
public:
///
/// \brief Construct Ipopt linearized static optimization
/// \param model The model
/// \param Q The position variables
/// \param Qdot The velocity variables
/// \param GeneralizedTorqueTarget Generalized torque target
/// \param activationInit Initial activation
/// \param useResidual If use residual (default: true)
/// \param pNormFactor p norm factor (default: 2)
/// \param verbose TODO: (default: 0)
/// \param eps EPS (default: 1e-10)
///
    StaticOptimizationIpoptLinearized(
            biorbd::Model& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueTarget,
            const biorbd::utils::Vector& activationInit,
            bool useResidual = true,
            unsigned int pNormFactor = 2,
            int verbose = 0,
            double eps = 1e-10);
///
/// \brief Destroy class properly
///
    virtual ~StaticOptimizationIpoptLinearized();

   ///
   /// \brief Method to return the constraint residuals
   /// \param n TODO:
   /// \param x TODO:  
   /// \param new_x TODO:
   /// \param m TODO:
   /// \param g TODO:
   /// \return True or False
   ///
    virtual bool eval_g(
       Ipopt::Index n,
       const Ipopt::Number* x,
       bool new_x,
       Ipopt::Index m,
       Ipopt::Number* g);

  
   ///
   /// \brief Method to return: 1) The structure of the jacobian (if "values" is NULL) 2) The values of the jacobian (if "values" is not NULL)
   /// \param n TODO:
   /// \param x TODO:  
   /// \param new_x TODO:
   /// \param m TODO:
    /// \param nele_jac TODO:
    /// \param iRow Number of rows
    /// \param jCol Number of columns
    /// \param values 
    /// \return True or False
    ///
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
    std::shared_ptr<biorbd::utils::Matrix> m_jacobian; ///< The Jacobian
    void prepareJacobian(); ///<Prepare the Jacobian

};

}}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
