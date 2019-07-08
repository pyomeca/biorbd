#ifndef IPOPTTEST_H
#define IPOPTTEST_H

#include <IpIpoptApplication.hpp>
#include "IpTNLP.hpp"
#include "biorbdConfig.h"
#include "s2mGenCoord.h"
#include "s2mMusculoSkeletalModel.h"
#include "s2mMuscles.h"
#include "s2mVector.h"
#include <Eigen/Dense>

class HS071_NLP: public Ipopt::TNLP
{
public:
   /** Default constructor */
   HS071_NLP(
           s2mMusculoSkeletalModel &model,
           unsigned int nTau,
           unsigned int nMus
           );

   /**@name Overloaded from TNLP */
   //@{
   /** Method to return some info about the NLP */
   virtual bool get_nlp_info(
      Ipopt::Index&          n,
      Ipopt::Index&          m,
      Ipopt::Index&          nnz_jac_g,
      Ipopt::Index&          nnz_h_lag,
      IndexStyleEnum& index_style
      );

   /** Method to return the bounds for my problem */
   virtual bool get_bounds_info(
      Ipopt::Index   n,
      Ipopt::Number* x_l,
      Ipopt::Number* x_u,
      Ipopt::Index   m,
      Ipopt::Number* g_l,
      Ipopt::Number* g_u
      );

   /** Method to return the starting point for the algorithm */
   virtual bool get_starting_point(
      Ipopt::Index   n,
      bool    init_x,
      Ipopt::Number* x,
      bool    init_z,
      Ipopt::Number* z_L,
      Ipopt::Number* z_U,
      Ipopt::Index   m,
      bool    init_lambda,
      Ipopt::Number* lambda
      );

   /** Method to return the objective value */
   virtual bool eval_f(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Number&       obj_value
      );

   /** Method to return the gradient of the objective */
   virtual bool eval_grad_f(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Number*       grad_f
      );

   /** Method to return the constraint residuals */
   virtual bool eval_g(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Index         m,
      Ipopt::Number*       g
      );

   /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
   virtual bool eval_jac_g(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Index         m,
      Ipopt::Index         nele_jac,
      Ipopt::Index*        iRow,
      Ipopt::Index*        jCol,
      Ipopt::Number*       values
      );

   /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
   virtual bool eval_h(
      Ipopt::Index         n,
      const Ipopt::Number* x,
      bool          new_x,
      Ipopt::Number        obj_factor,
      Ipopt::Index         m,
      const Ipopt::Number* lambda,
      bool          new_lambda,
      Ipopt::Index         nele_hess,
      Ipopt::Index*        iRow,
      Ipopt::Index*        jCol,
      Ipopt::Number*       values
      );

   /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
   virtual void finalize_solution(
      Ipopt::SolverReturn               status,
      Ipopt::Index                      n,
      const Ipopt::Number*              x,
      const Ipopt::Number*              z_L,
      const Ipopt::Number*              z_U,
      Ipopt::Index                      m,
      const Ipopt::Number*              g,
      const Ipopt::Number*              lambda,
      Ipopt::Number                     obj_value,
      const Ipopt::IpoptData*           ip_data,
      Ipopt::IpoptCalculatedQuantities* ip_cq
      );
   //@}

protected:
   unsigned int m_nQ;
   unsigned int m_nQdot;
   unsigned int m_nMus;
   unsigned int m_nDof;
   unsigned int m_nTau;
   s2mTau m_tau_kin;
   s2mVector m_activationInit;
   s2mVector m_activation;
   s2mVector m_residual;
   unsigned int m_p;
   s2mGenCoord m_Q;
   s2mGenCoord m_Qdot;
   s2mGenCoord m_Qddot;
   s2mMusculoSkeletalModel &m_model;
   double m_eps;
   std::vector<s2mMuscleStateActual> m_State;
   double m_ponderation;

   void dispatch(
           const Ipopt::Number* x
           );

private:

   HS071_NLP(
      const HS071_NLP&
      );

   HS071_NLP& operator=(
      const HS071_NLP&
      );

};

#endif // IPOPTTEST_H
