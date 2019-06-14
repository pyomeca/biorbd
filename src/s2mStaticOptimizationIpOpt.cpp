#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimizationIpOpt.h"


s2mStaticOptimizationIpOpt::s2mStaticOptimizationIpOpt(s2mMusculoSkeletalModel &model,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mTau& tau_init,
        const s2mVector &activationInit,
        unsigned int p,
        const unsigned int epsilon
        ) :
    m_nQ(model.nbQ()),
    m_nQdot(model.nbQdot()),
    m_nMus(model.nbMuscleTotal()),
    m_nDof(model.nbDof()),
    m_nTau(model.nbTau()),
    m_tau(tau_init),
    m_activationInit(activationInit),
    m_activation(s2mVector(m_nMus)),
    m_p(p),
    m_Q(Q),
    m_Qdot(Qdot),
    m_model(model),
    m_epsilon(epsilon)
{
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
}

s2mStaticOptimizationIpOpt::s2mStaticOptimizationIpOpt(
        s2mMusculoSkeletalModel &model,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mTau& tau_init,
        unsigned int p,
        const unsigned int epsilon
        ) :
    m_nQ(m_model.nbQ()),
    m_nQdot(m_model.nbQdot()),
    m_nMus(m_model.nbMuscleTotal()),
    m_nDof(m_model.nbDof()),
    m_nTau(m_model.nbTau()),
    m_tau(tau_init),
    m_activationInit(s2mVector(m_nMus)),
    m_activation(s2mVector(m_nMus)),
    m_p(p),
    m_Q(Q),
    m_Qdot(Qdot),
    m_model(model),
    m_epsilon(epsilon)
{
    model.updateMuscles(model, Q, Qdot, true);
}

s2mStaticOptimizationIpOpt::~s2mStaticOptimizationIpOpt()
{

}

bool s2mStaticOptimizationIpOpt::get_nlp_info(
        Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
{
    n = m_nMus;
    m = m_nTau;
    nnz_jac_g = n*m;
    nnz_h_lag = m*m;
    index_style = TNLP::C_STYLE;
}

bool s2mStaticOptimizationIpOpt::get_bounds_info(
        Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u, Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    for( Ipopt::Index i = 0; i < n; i++ )
       {
          x_l[i] = 0.01;
       }
    for( Ipopt::Index i = 0; i < n; i++ )
       {
          x_u[i] = 1.0;
       }
    for( Ipopt::Index i = 0; i < m; i++ )
       {
        g_l[i] = g_u[i] = m_tau[i];
       }
    return true;
}

bool s2mStaticOptimizationIpOpt::get_starting_point(
        Ipopt::Index n, bool init_x, Ipopt::Number *x, bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    fillActivation(n, x);

    for( Ipopt::Index i = 0; i < m; i++ )
       {
        x[i] = m_activation[i];
       }

    return true;
}

bool s2mStaticOptimizationIpOpt::eval_f(
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value)
{
    fillActivation(n, x);
    obj_value = m_activation.norm(m_p);
    return true;
}

bool s2mStaticOptimizationIpOpt::eval_grad_f(
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    fillActivation(n, x);
    s2mVector grad(m_activation.grad_norm(m_p));
    for( Ipopt::Index i = 0; i < n; i++ )
       {
        grad_f[i] = grad[i];
       }
    return true;
}

bool s2mStaticOptimizationIpOpt::eval_g(
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    fillActivation(n, x);
    std::vector<s2mMuscleStateActual> state;// controls
    for (unsigned int i = 0; i<m_nMus; ++i)
        state.push_back(s2mMuscleStateActual(0, m_activation[i]));

    // Compute the torques from muscles
    s2mTau tau_calcul = m_model.muscularJointTorque(m_model, state, true, &m_Q, &m_Qdot);

    for( Ipopt::Index i = 0; i < m; i++ )
       {
        g[i] = tau_calcul[i];
       }
    return true;
}

bool s2mStaticOptimizationIpOpt::eval_jac_g(
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    fillActivation(n, x);

    // controls
    std::vector<s2mMuscleStateActual> state;
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::vector<s2mMuscleStateActual> state;
        state.push_back(s2mMuscleStateActual(0, m_activation[i]));
    }
    s2mTau tau_calcul_actual = m_model.muscularJointTorque(m_model, state, true, &m_Q, &m_Qdot);
    unsigned int k(0);
    for( Ipopt::Index j = 0; j < n; j++ )
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
            state_epsilon.push_back(s2mMuscleStateActual(0, m_activation[i]+delta*m_epsilon));
        }
        s2mTau tau_calcul_epsilon = m_model.muscularJointTorque(m_model, state_epsilon, true, &m_Q, &m_Qdot);
        for( Ipopt::Index i = 0; i < m; i++ )
        {
            values[k] = (tau_calcul_actual[i]-tau_calcul_epsilon[i])/m_epsilon;
            k++;
       }
    }
    return true;
    }


bool s2mStaticOptimizationIpOpt::eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    return false;
}

void s2mStaticOptimizationIpOpt::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x, const Ipopt::Number *z_L, const Ipopt::Number *z_U, Ipopt::Index m, const Ipopt::Number *g, const Ipopt::Number *lambda, Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data, Ipopt::IpoptCalculatedQuantities *ip_cq)
{
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

void s2mStaticOptimizationIpOpt::fillActivation(Ipopt::Index n, const Ipopt::Number *x)
{
    for( Ipopt::Index i = 0; i < n; i++ )
    {
        m_activation[i] = x[i];
    }
}




