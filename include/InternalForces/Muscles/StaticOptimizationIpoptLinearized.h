#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H

#include "biorbdConfig.h"

#include "InternalForces/Muscles/StaticOptimizationIpopt.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class Matrix;
class Vector;
}  // namespace utils

namespace internal_forces {
namespace muscles {
///
/// \brief The actual implementation of the Static Optimization problem using a
/// linearized approach
///
/// This algo should be much faster than tradition but less precise. The
/// torque of each muscle is linearized between no activation and its initial
/// activation (the solution of the previous frame), so the approximation is
/// exact at that reference activation (Michaud, "Two efficient static
/// optimization algorithms that account for muscle-tendon equilibrium:
/// approaching the constraint Jacobian via a constant or a cubic spline
/// function")
///
class BIORBD_API StaticOptimizationIpoptLinearized
    : public StaticOptimizationIpopt {
 public:
  ///
  /// \brief Construct an Ipopt static optimization problem
  /// \param model The musculoskeletal Model
  /// \param Q The generalized coordinates
  /// \param Qdot The generalized velocities
  /// \param torqueTarget The generalized torque target
  /// \param activationInit The initial activation, also used as the
  /// reference activation of the linearization
  /// \param useResidual If use residual torque, if set to false, the
  /// optimization will fail if the model is not strong enough
  /// \param pNormFactor The p-norm to perform
  /// \param verbose Level of IPOPT verbose you want
  /// \param eps The precision to perform the finite diffentiation
  ///
  StaticOptimizationIpoptLinearized(
      Model& model,
      const rigidbody::GeneralizedCoordinates& Q,
      const rigidbody::GeneralizedVelocity& Qdot,
      const rigidbody::GeneralizedTorque& torqueTarget,
      const utils::Vector& activationInit,
      bool useResidual = true,
      unsigned int pNormFactor = 2,
      int verbose = 0,
      double eps = 1e-8);

  ///
  /// \brief Destroy class properly
  ///
  virtual ~StaticOptimizationIpoptLinearized();

  ///
  /// \brief Method to return the constraint residuals
  /// \param n The number of variables
  /// \param x The values of the variables
  /// \param new_x If the variables were modified by IPOPT
  /// \param m The number of constraints
  /// \param g The actual contraints residual
  /// \return Return the presence of that function
  ///
  virtual bool eval_g(
      Ipopt::Index n,
      const Ipopt::Number* x,
      bool new_x,
      Ipopt::Index m,
      Ipopt::Number* g);

  ///
  /// \brief Return the jacobian of contraints
  /// \param n The number of variables
  /// \param x The values of the variables
  /// \param new_x If the variables were modified by IPOPT
  /// \param m The number of constraints
  /// \param nele_jac Number of elements in the jacobian matrix
  /// \param iRow iterator on the rows of the jacobian
  /// \param jCol iterator on the columns of the jacobian
  /// \param values The actual constraint jacobian
  /// \return Return the presence of that function
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
  std::shared_ptr<utils::Matrix> m_jacobian;  ///< The constraints jacobian
  std::shared_ptr<utils::Vector>
      m_torquePassive;  ///< The muscle torque when not activated
  void prepareJacobian();  ///< Setup the constant constraints jacobian
};

}  // namespace muscles
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_LINEARIZED_H
