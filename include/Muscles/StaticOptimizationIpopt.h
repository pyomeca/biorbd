#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_IPOPT_H

#include <memory>
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"

namespace biorbd {
class Model;

namespace utils {
class Vector;
}

namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedTorque;
}

namespace muscles {
class StateDynamics;
///
/// \brief Class StaticOptimizationIpopt
///
class BIORBD_API StaticOptimizationIpopt : public Ipopt::TNLP
{
public:
///
/// \brief Construct an Ipopt static optimization
/// \param model The model
/// \param Q The position variables
/// \param Qdot The velocity variables
/// \param GeneralizedTorqueTarget The generalized torque target
/// \param activationInit The initial activation (vector)
/// \param useResidual If use residual torque (default: true)
/// \param pNormFactor The norm factor (default: 2)
/// \param verbose (default: 0)
/// \param eps EPS (default: 1e-10)
///
    StaticOptimizationIpopt(
            biorbd::Model &model,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
            const biorbd::rigidbody::GeneralizedTorque &GeneralizedTorqueTarget,
            const biorbd::utils::Vector &activationInit,
            bool  useResidual = true,
            unsigned int pNormFactor = 2,
            int verbose = 0,
            double eps = 1e-10);

///
/// \brief Destroy class properly
///
    virtual ~StaticOptimizationIpopt();

    // Method to return some info about the NLP
   ///
   /// \brief Get info about the NLP
   /// \param n Index n of the ipopt static optimization
   /// \param m Index m of the ipopt static optimization   
   /// \param nnz_jac_g TODO: 
   /// \param nnz_h_lag TODO:
   /// \param index_style TODO:
   /// \return True or false
   ///
    virtual bool get_nlp_info(
            Ipopt::Index& n,
            Ipopt::Index& m,
            Ipopt::Index& nnz_jac_g,
            Ipopt::Index& nnz_h_lag,
            IndexStyleEnum& index_style);

    // Method to return the bounds for my problem
   ///
   /// \brief Return the bounds for my problem
   /// \param n Index n of the ipopt static optimization
   /// \param x_l TODO:  
   /// \param x_u TODO:
   /// \param m Index m
   /// \param g_l TODO:
   /// \param g_u TODO:
   /// \return True or False
   ///
    virtual bool get_bounds_info(
            Ipopt::Index n,
            Ipopt::Number* x_l,
            Ipopt::Number* x_u,
            Ipopt::Index m,
            Ipopt::Number* g_l,
            Ipopt::Number* g_u);

    // Method to return the starting point for the algorithm
   ///
   /// \brief Return the starting point for the algorithm
   /// \param n Index n of the ipopt static optimization
   /// \param init_x TODO:  
   /// \param x TODO:
   /// \param init_z TODO:
   /// \param z_L TODO:
   /// \param z_U TODO:
   /// \param m TODO:
   /// \param init_lambda TODO:
   /// \param lambda TODO: 
   /// \return True or False
   ///
    virtual bool get_starting_point(
            Ipopt::Index n,
            bool init_x,
            Ipopt::Number* x,
            bool init_z,
            Ipopt::Number* z_L,
            Ipopt::Number* z_U,
            Ipopt::Index m,
            bool init_lambda,
            Ipopt::Number* lambda);

    // Method to return the objective value
    ///
    /// \brief To return the objective value
    /// \param n TODO:
    /// \param x TODO:
    /// \param new_x TODO:
    /// \param obj_value: 
    /// \return True or False
    ///
    virtual bool eval_f(
            Ipopt::Index n,
            const Ipopt::Number* x,
            bool new_x,
            Ipopt::Number& obj_value);

    // Method to return the gradient of the objective
    ///
    /// \brief Return the gradient of the objective
    /// \param n TODO:
    /// \param x TODO:
    /// \param new_x TODO:
    /// \param grad_f TODO:
    /// \return True or False
    ///
    virtual bool eval_grad_f(
            Ipopt::Index n,
            const Ipopt::Number* x,
            bool new_x,
            Ipopt::Number* grad_f);

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
    /// \brief Method to return: 1) The structure of the jacobian (if "values" is NULL), 2) The values of the jacobian (if "values" is not NULL)
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

///
    /// \brief Return the final solution
    /// \param status The status
   /// \param n Index n of the ipopt static optimization
   /// \param x TODO:
   /// \param z_L TODO:
   /// \param z_U TODO:
   /// \param m TODO:
   /// \param g TODO:
   /// \param lambda TODO: 
   /// \param obj_value TODO: 
   /// \param ip_data TODO:
   /// \param ip_cq TODO: 
   ///
    virtual void finalize_solution(
            Ipopt::SolverReturn status,
            Ipopt::Index n,
            const Ipopt::Number* x,
            const Ipopt::Number* z_L,
            const Ipopt::Number* z_U,
            Ipopt::Index m,
            const Ipopt::Number* g,
            const Ipopt::Number* lambda,
            Ipopt::Number obj_value,
            const Ipopt::IpoptData* ip_data,
            Ipopt::IpoptCalculatedQuantities* ip_cq);

///
/// \brief Return the final solution
/// \return The final solution
///
    biorbd::utils::Vector finalSolution() const;
    
///
/// \brief Return the final residual
/// \return The final residual
///
    biorbd::utils::Vector finalResidual() const;




protected:
    biorbd::Model& m_model; ///< The model
    std::shared_ptr<unsigned int> m_nbQ; ///< The number of position variables
    std::shared_ptr<unsigned int> m_nbQdot; ///< The number of velocity variables
    std::shared_ptr<unsigned int> m_nbMus;///< The number of muscles
    std::shared_ptr<unsigned int> m_nbDof; ///< The number of degrees of freedom
    std::shared_ptr<unsigned int> m_nbGeneralizedTorque; ///< The number of generalized torques 
    std::shared_ptr<unsigned int> m_nbGeneralizedTorqueResidual; ///< The number of Generalized torque residual
    std::shared_ptr<double> m_eps; ///< eps
    std::shared_ptr<biorbd::utils::Vector> m_activations; ///< The activations
    std::shared_ptr<biorbd::rigidbody::GeneralizedCoordinates> m_Q; ///< The position variables
    std::shared_ptr<biorbd::rigidbody::GeneralizedCoordinates> m_Qdot; ///<The velocity variables 
    std::shared_ptr<biorbd::rigidbody::GeneralizedTorque> m_GeneralizedTorqueTarget; ///< The generalized torque target 
    std::shared_ptr<biorbd::utils::Vector> m_GeneralizedTorqueResidual; ///< The generalized torque residual
    std::shared_ptr<double> m_GeneralizedTorquePonderation; ///< The generalized torque ponderation
    std::shared_ptr<std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>>> m_states; ///< The states
    std::shared_ptr<unsigned int> m_pNormFactor; ///< The p norm factor
    std::shared_ptr<int> m_verbose; ///< Verbose
    std::shared_ptr<biorbd::utils::Vector> m_finalSolution; ///< The final solution
    std::shared_ptr<biorbd::utils::Vector> m_finalResidual; ///< The final residual

    ///
    /// \brief To dispatch
    /// \param x TODO:
    ///
    void dispatch(const Ipopt::Number* x);

};

}}

#endif // BIORBD_MUSCLES_OPTIMIZATION_IPOPT_H
