#define BIORBD_API_EXPORTS
#include "../include/IpoptTest.h"

#include <cassert>
#include <iostream>

// constructor
HS071_NLP::HS071_NLP(s2mMusculoSkeletalModel &model,
                     const s2mGenCoord &Q,
                     const s2mGenCoord &Qdot,
                     const s2mTau &tauTarget,
                     bool useResidual,
                     int verbose):
    m_model(model),
    m_nQ(m_model.nbQ()),
    m_Q(Q),
    m_Qdot(Qdot),
    m_nTau(m_model.nbTau()),
    m_nTauResidual(m_nQ),
    m_tauTarget(tauTarget),
    m_tauResidual(Eigen::VectorXd::Zero(m_nTau)),
    m_tauPonderation(1000),
    m_nMus(m_model.nbMuscleTotal()),
    m_activations(Eigen::VectorXd::Ones(m_nMus) * 0.01),
    m_states(std::vector<s2mMuscleStateActual>(m_nMus)),
    m_pNormFactor(2),
    m_eps(1e-10),
    m_verbose(verbose)
{
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    if (!useResidual){
        m_tauResidual.setZero();
        m_nTauResidual = 0;
        m_tauPonderation = 0;
    }
}


bool HS071_NLP::get_nlp_info(
   Ipopt::Index& n,
   Ipopt::Index& m,
   Ipopt::Index& nnz_jac_g,
   Ipopt::Index& nnz_h_lag,
   IndexStyleEnum& index_style
   )
{
    index_style = TNLP::C_STYLE;

    // variables
    n = static_cast<int>(m_nMus + m_nTauResidual);

    // Constraints
    m = static_cast<int>(m_nTau);
    if (m_nTauResidual)
        nnz_jac_g = (static_cast<int>(m_nMus) + 1) * static_cast<int>(m_nTau);
    else
        nnz_jac_g = static_cast<int>(m_nMus) * static_cast<int>(m_nTau);
    nnz_h_lag = static_cast<int>(m_nTau) * static_cast<int>(m_nTau);

    if (m_verbose >= 2){
        std::cout << "n: " << n << std::endl;
        std::cout << "m: " << m << std::endl;
    }
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
    // Should not be necessary?
    assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
    assert(static_cast<unsigned int>(m) == m_nTau);

    // Variables
    for( unsigned int i = 0; i < m_nMus; ++i ){
        x_l[i] = 0.0001;
        x_u[i] = 0.9999;
    }
    for( unsigned int i = m_nMus; i < m_nMus+m_nTauResidual; ++i ){
        x_l[i] = -1000.0;
        x_u[i] = 1000.0;
    }

    // Constraints
    for( unsigned int i = 0; i < m_nTau; ++i )
        g_l[i] = g_u[i] = 0;

    return true;
}

bool HS071_NLP::get_starting_point(
        Ipopt::Index,
        bool init_x,
        Ipopt::Number* x,
        bool init_z,
        Ipopt::Number*,
        Ipopt::Number*,
        Ipopt::Index,
        bool init_lambda,
        Ipopt::Number*)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    for( unsigned int i = 0; i < m_nMus; i++ )
        x[i] = m_activations[i];

    for( unsigned int i = 0; i < m_nTauResidual; i++ )
        x[i+m_nMus] = m_tauResidual[i];

    if (m_verbose >= 2){
        std::cout << std::endl << "Initial guesses" << std::endl;
        std::cout << "Activations = " << m_activations.transpose() << std::endl;
        if (m_nTauResidual)
            std::cout << "Residuals = " << m_tauResidual.transpose() << std::endl;
    }

    return true;
}

bool HS071_NLP::eval_f(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Number& obj_value)
{
    assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
    if (new_x)
        dispatch(x);

    // Warning, residual norm HAS to be a non-sqrt, otherwise the problem is degenerated. The activation is just a speed matter.
    obj_value = m_activations.norm(m_pNormFactor, true) + m_tauPonderation*m_tauResidual.norm(2, true);

    return true;
}

bool HS071_NLP::eval_grad_f(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Number* grad_f){
   assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
   if (new_x)
       dispatch(x);

   s2mVector grad_activ(m_activations.norm_gradient(m_pNormFactor, true));
   s2mVector grad_residual(m_tauResidual.norm_gradient(2, true));

   for( unsigned i = 0; i < m_nMus; i++ )
       grad_f[i] = grad_activ[i];
   for( unsigned int i = 0; i < m_nTauResidual; i++ )
       grad_f[i+m_nMus] = m_tauPonderation*grad_residual[i];
   return true;
}

bool HS071_NLP::eval_g(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Number* g)
{
   assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
   assert(static_cast<unsigned int>(m) == m_nTau);
   if (new_x)
       dispatch(x);

   s2mTau tauMusc = m_model.muscularJointTorque(m_model, m_states, false, &m_Q, &m_Qdot);

   // TODO : adjust dimensions for when "root_actuated" is set to false in bioMod file
   for( Ipopt::Index i = 0; i < m; i++ )
        g[i] = tauMusc[i] + m_tauResidual[i] - m_tauTarget[i];

   if (m_verbose >= 2){
       std::cout << "tau_musc = " << tauMusc.transpose() << std::endl;
       std::cout << "tau_residual = " << m_tauResidual.transpose() << std::endl;
   }
   return true;
}

bool HS071_NLP::eval_jac_g(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index,
        Ipopt::Index* iRow,
        Ipopt::Index* jCol,
        Ipopt::Number* values)
{
    if (values == nullptr) {
        // Setup non-zeros values
        Ipopt::Index k(0);
        for (Ipopt::Index j = 0; j < n-m; ++j){
            for (Ipopt::Index i = 0; i<m; ++i){
                iRow[k] = i;
                jCol[k++] = j;
            }
        }
        for (Ipopt::Index j = 0; j < m; ++j ){
            iRow[k] = j;
            jCol[k++] = j+n-m;
        }
    } else {
        if (new_x)
            dispatch(x);
        s2mTau tauMusc = m_model.muscularJointTorque(m_model, m_states, false, &m_Q, &m_Qdot);
        unsigned int k(0);
        for( unsigned int j = 0; j < m_nMus; ++j ){
            std::vector<s2mMuscleStateActual> stateEpsilon;
            for (unsigned int i = 0; i < m_nMus; ++i){
                unsigned int delta(0);
                if (i == j)
                    delta = 1;
                stateEpsilon.push_back(s2mMuscleStateActual(0, m_activations[i]+delta*m_eps));
            }
            s2mTau tauCalculEpsilon = m_model.muscularJointTorque(m_model, stateEpsilon, false, &m_Q, &m_Qdot);
            for( Ipopt::Index i = 0; i < m; i++ )
                values[k++] = (tauCalculEpsilon[i]-tauMusc[i])/m_eps;
        }
        for( unsigned int j = 0; j < m_nTauResidual; j++ )
            values[k++] = 1;

        if (m_verbose >= 2){
            k = 0;
            s2mMatrix jacobian(m_nTau, static_cast<unsigned int>(n));
            jacobian.setZero();
            for( unsigned int j = 0; j < m_nMus; j++ )
                for( Ipopt::Index i = 0; i < m; i++ )
                    jacobian(i,j) = values[k++];
            for( unsigned int j = 0; j < m_nTauResidual; j++ ){
                jacobian(j, j+m_nMus) = values[k++];
            }
            std::cout << "jacobian = " << std::endl << jacobian << std::endl;
        }

    }
   return true;
}

void HS071_NLP::finalize_solution(
        Ipopt::SolverReturn,
        Ipopt::Index,
        const Ipopt::Number* x,
        const Ipopt::Number*, // z_L,
        const Ipopt::Number*, // z_U,
        Ipopt::Index,
        const Ipopt::Number*,
        const Ipopt::Number*,
        Ipopt::Number obj_value,
        const Ipopt::IpoptData*,
        Ipopt::IpoptCalculatedQuantities*)
{
    // Storing to solution
    dispatch(x);

    // Plot it, if it makes sense
    if (m_verbose >= 1){
        std::cout << std::endl << "Final results" << std::endl;
        std::cout << "f(x*) = " << obj_value << std::endl;
        std::cout << "Activations = " << m_activations.transpose() << std::endl;
        std::cout << "Muscular torques = " << m_model.muscularJointTorque(m_model, m_states, false, &m_Q, &m_Qdot).transpose() << std::endl;
        if (m_nTauResidual){
            std::cout << "Residual torques= " << m_tauResidual.transpose() << std::endl;
        }
//        // Uncomment to show lagrange multipliers
//        std::cout << "Solution of the bound multipliers, z_L and z_U" << std::endl;
//        for( Ipopt::Index i = 0; i < n; i++ )
//            std::cout << "z_L[" << i << "] = " << z_L[i] << ", z_U[" << i << "] = " << z_U[i] << std::endl << std::endl;
    }
}

void HS071_NLP::dispatch(const Ipopt::Number *x)
{
    for(unsigned int i = 0; i < m_nMus; i++ ){
        m_activations[i] = x[i];
        m_states[i].setActivation(m_activations[i]);
    }

    for(unsigned int i = 0; i < m_nTauResidual; i++ )
        m_tauResidual[i] = x[i+m_nMus];
}
