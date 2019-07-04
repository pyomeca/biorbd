#define BIORBD_API_EXPORTS
#include "../include/IpoptTest.h"

#include <cassert>
#include <iostream>

// constructor
HS071_NLP::HS071_NLP(
        s2mMusculoSkeletalModel &model,
        unsigned int nTau,
        unsigned int nMus
        ):
    m_nQ(nTau),
    m_nQdot(nTau),
    m_nMus(nMus),
    m_nDof(nTau),
    m_nTau(nTau),
    m_tau(s2mTau(s2mVector(nTau))),
    m_activationInit(s2mVector(nMus)),
    m_activation(s2mVector(nMus)),
    m_residual(s2mVector(nTau)),
    m_p(2),
    m_Q(s2mGenCoord(s2mVector(nTau))),
    m_Qdot(s2mGenCoord(s2mVector(nTau))),
    m_Qddot(s2mGenCoord(s2mVector(nTau))),
    m_model(model),
    m_eps(1e-10),
    m_State(std::vector<s2mMuscleStateActual>(nMus)),
    m_ponderation(10000)
{
    m_Q.setZero();
    m_Qdot.setZero();
    m_Qddot.setZero();
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    m_residual.setOnes();
    m_tau.setZero();
}

// destructor
HS071_NLP::~HS071_NLP()
{

}

bool HS071_NLP::get_nlp_info(
   Ipopt::Index&          n,
   Ipopt::Index&          m,
   Ipopt::Index&          nnz_jac_g,
   Ipopt::Index&          nnz_h_lag,
   IndexStyleEnum& index_style
   )
{
   n = static_cast<int>(m_nMus) + static_cast<int>(m_nTau);
   std::cout << "n: " << n << std::endl;
   m = static_cast<int>(m_nTau);
   std::cout << "m: " << m << std::endl;
   nnz_jac_g = (static_cast<int>(m_nMus) + 1) * static_cast<int>(m_nTau);
   nnz_h_lag = static_cast<int>(m_nTau) * static_cast<int>(m_nTau);

   index_style = TNLP::C_STYLE;

   return true;
}

bool HS071_NLP::get_bounds_info(
   Ipopt::Index   n,
   Ipopt::Number* x_l,
   Ipopt::Number* x_u,
   Ipopt::Index   m,
   Ipopt::Number* g_l,
   Ipopt::Number* g_u
   )
{
    assert(n == static_cast<int>(m_nMus) + static_cast<int>(m_nTau));
    assert(m == static_cast<int>(m_nTau));

   for( Ipopt::Index i = 0; i < static_cast<int>(m_nMus); i++ )
   {
      x_l[i] = 0.01;
      x_u[i] = 0.99;
   }

   for( Ipopt::Index i = static_cast<int>(m_nMus); i < static_cast<int>(m_nMus+m_nTau); i++ )
   {
      x_l[i] = -1000.0;
      x_u[i] = 1000.0;
   }

   for( Ipopt::Index i = 0; i < m; i++ )
      {
       g_l[i] = g_u[i] = 0;
      }
   return true;
}

bool HS071_NLP::get_starting_point(
   Ipopt::Index   n,
   bool    init_x,
   Ipopt::Number* x,
   bool    init_z,
   Ipopt::Number* z_L,
   Ipopt::Number* z_U,
   Ipopt::Index   m,
   bool    init_lambda,
   Ipopt::Number* lambda
   )
{
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   for( Ipopt::Index i = 0; i < m_nMus; i++ )
      {
       x[i] = m_activationInit[i];
       std::cout << "m_activationInit[" << i << "]: " << x[i] << std::endl;
      }
   for( Ipopt::Index i = 0; i < m; i++ )
      {
       x[i+n-m] = 0.1;
      }

   return true;
}

bool HS071_NLP::eval_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number&       obj_value
   )
{
    assert(n == static_cast<int>(m_nMus) + static_cast<int>(m_nTau));
    if (new_x){
        dispatch(x);
    }

   obj_value = m_activation.norm(m_p) + m_ponderation*m_residual.norm(2);

   return true;
}

bool HS071_NLP::eval_grad_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number*       grad_f
   )
{
   assert(static_cast<unsigned int>(n) == m_nTau+m_nMus);

   s2mVector grad_activ(m_activation.grad_norm(m_p));
   s2mVector grad_residual(m_residual.grad_norm(2));

   for( Ipopt::Index i = 0; i < static_cast<int>(m_nMus); i++ )
      {
       grad_f[i] = grad_activ[i];
      }
   for( Ipopt::Index i = 0; i < static_cast<int>(m_nTau); i++ ){
       grad_f[i+static_cast<int>(m_nMus)] = m_ponderation*grad_residual[i];
   }

   return true;
}

bool HS071_NLP::eval_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Index         m,
   Ipopt::Number*       g
   )
{
   assert(static_cast<unsigned int>(n) == m_nTau+m_nMus);
   assert(static_cast<unsigned int>(m) == m_nTau);

   if (new_x){
       dispatch(x);
   }

   for (unsigned int i = 0; i<m_nMus; ++i){
       m_State[i].setActivation(m_activation[i]);
   }
   //s2mTau tau_inv = m_model.muscularJointTorque(m_model, m_State, true, &m_Q, &m_Qdot);
   s2mTau tau_inv(m_model.nbTau());
   tau_inv.setZero();
   RigidBodyDynamics::InverseDynamics(m_model, m_Q, m_Qdot, m_Qddot, tau_inv);


   for( Ipopt::Index i = 0; i < m; i++ )
      {
       g[i] = tau_inv[i] + m_residual[i] - m_tau[i];
       std::cout << "tau_calcul[" << i << "]: " << tau_inv[i] << std::endl;
      }


   return true;
}

bool HS071_NLP::eval_jac_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Index         m,
   Ipopt::Index         nele_jac,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
   )
{
    if (values == nullptr) {
        unsigned int k(0);
        for (unsigned int j = 0; static_cast<int>(j) < n-m; ++j){
            for (unsigned int i = 0; i<m; ++i){
                iRow[k] = static_cast<int>(i);
                jCol[k] = static_cast<int>(j);
                ++k;
            }
        }
        for (unsigned  int j = 0; j < m; j++ ){
            iRow[k] = static_cast<int>(j);
            jCol[k] = static_cast<int>(j+n-m);
            ++k;
        }

    }
    else
    {
        for (unsigned int i = 0; i<m_nMus; ++i){
            m_State[i].setActivation(m_activation[i]);
        }
        s2mTau tau_calcul_actual = m_model.muscularJointTorque(m_model, m_State, true, &m_Q, &m_Qdot);
        s2mMatrix jacobian(m_nTau, static_cast<unsigned int>(n));
        jacobian.setZero();
        unsigned int k(0);
        for( unsigned int j = 0; static_cast<unsigned int>(j) < m_nMus; j++ )
           {
            std::vector<s2mMuscleStateActual> state_epsilon;
            for (unsigned int i = 0; i<m_nMus; ++i){
                unsigned int delta;
                if (i == j){
                    delta = 1;
                }
                else {
                    delta = 0;
                }
                state_epsilon.push_back(s2mMuscleStateActual(0, m_activation[i]+delta*m_eps));
            }
            s2mTau tau_calcul_epsilon = m_model.muscularJointTorque(m_model, state_epsilon, true, &m_Q, &m_Qdot);
            for( Ipopt::Index i = 0; i < m; i++ )
            {
                values[k] = (tau_calcul_epsilon[i]-tau_calcul_actual[i])/m_eps;
                std::cout.precision(20);
                jacobian(i,j) = values[k];
                k++;
           }
        }
        for( Ipopt::Index j = 0; j < static_cast<int>(m_nTau); j++ ){
            values[k] = 1;
            jacobian(j, j+static_cast<int>(m_nMus)) = values[k];
            k++;
        }
    }

   return true;
}
// [TNLP_eval_jac_g]

// [TNLP_eval_h]
//return the structure or values of the Hessian
bool HS071_NLP::eval_h(
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
   )
{
   return false;
}
// [TNLP_eval_h]

// [TNLP_finalize_solution]
void HS071_NLP::finalize_solution(
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
   )
{
   // here is where we would store the solution to variables, or write to a file, etc
   // so we could use the solution.

   // For this example, we write the solution to the console
   std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
   for( Ipopt::Index i = 0; i < n; i++ )
   {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
   for( Ipopt::Index i = 0; i < n; i++ )
   {
      std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
   }
   for( Ipopt::Index i = 0; i < n; i++ )
   {
      std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
   }

   std::cout << std::endl << std::endl << "Objective value" << std::endl;
   std::cout << "f(x*) = " << obj_value << std::endl;

   std::cout << std::endl << "Final value of the constraints:" << std::endl;
   for( Ipopt::Index i = 0; i < m; i++ )
   {
      std::cout << "g(" << i << ") = " << g[i] << std::endl;
   }
}
// [TNLP_finalize_solution]

void HS071_NLP::dispatch(const Ipopt::Number *x)
{
    for(int i = 0; i < static_cast<int>(m_nMus); i++ )
    {
        m_activation[i] = x[i];
    }
    for(int i = 0; i < static_cast<int>(m_nTau); i++ )
    {
        m_residual[i] = x[i+static_cast<int>(m_nMus)];
        std::cout << "m_residual[" << i << "]: " << m_residual[i] << std::endl;
    }

}
