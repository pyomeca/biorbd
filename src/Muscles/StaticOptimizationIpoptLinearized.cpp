#define BIORBD_API_EXPORTS
#include "Muscles/StaticOptimizationIpoptLinearized.h"

#include "s2mMusculoSkeletalModel.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::StaticOptimizationIpoptLinearized::StaticOptimizationIpoptLinearized(
        s2mMusculoSkeletalModel &model,
        const biorbd::utils::GenCoord &Q,
        const biorbd::utils::GenCoord &Qdot,
        const biorbd::utils::Tau &tauTarget,
        const biorbd::utils::Vector &activationInit,
        bool useResidual,
        unsigned int pNormFactor,
        int verbose,
        double eps
        ) :
    biorbd::muscles::StaticOptimizationIpopt(model, Q, Qdot, tauTarget, activationInit, useResidual, pNormFactor, verbose, eps),
    m_jacobian(biorbd::utils::Matrix(m_nDof, m_nMus))
{
    prepareJacobian();
}


void biorbd::muscles::StaticOptimizationIpoptLinearized::prepareJacobian()
{
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    std::vector<biorbd::muscles::StateDynamics> state_zero;
    for (unsigned int i = 0; i<m_nMus; ++i){
        state_zero.push_back(biorbd::muscles::StateDynamics(0, 0));
    }
    biorbd::utils::Tau tau_zero(m_model.muscularJointTorque(m_model, state_zero, false, &m_Q, &m_Qdot));
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::vector<biorbd::muscles::StateDynamics> state;
        for (unsigned int j = 0; j<m_nMus; ++j){
            unsigned int delta;
            if (j == i){
                delta = 1;
            }
            else {
                delta = 0;
            }
            state.push_back(biorbd::muscles::StateDynamics(0, delta*1));
        }
        biorbd::utils::Tau tau(m_model.muscularJointTorque(m_model, state, true, &m_Q, &m_Qdot));
        for (unsigned int j = 0; j<m_nTau; ++j){
            m_jacobian(j, i) = tau(j)-tau_zero(j);
        }
    }
}

biorbd::muscles::StaticOptimizationIpoptLinearized::~StaticOptimizationIpoptLinearized()
{

}

bool biorbd::muscles::StaticOptimizationIpoptLinearized::eval_g(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Number *g)
{
    assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
    assert(static_cast<unsigned int>(m) == m_nTau);
    if (new_x){
        dispatch(x);
    }
    std::vector<biorbd::muscles::StateDynamics> state;// controls
    for (unsigned int i = 0; i<m_nMus; ++i){
        state.push_back(biorbd::muscles::StateDynamics(0, m_activations[i]));
    }
    // Compute the torques from muscles
    biorbd::utils::Tau tau_calcul(m_model.muscularJointTorque(m_model, state, false, &m_Q, &m_Qdot));
    biorbd::utils::Vector res(static_cast<unsigned int>(m));
    for( Ipopt::Index i = 0; i < m; i++ )
       {
        for (unsigned int j = 0; j<m_nMus; j++){
            res[i] += m_jacobian(i,j)*x[j];
        }
        g[i] = res[i] + m_tauResidual[i];
       }
    if (m_verbose >= 2){
        std::cout << "tau_musc_approximated = " << res.transpose() << std::endl;
        std::cout << "tau_residual = " << m_tauResidual.transpose() << std::endl;
    }
    return true;
}

bool biorbd::muscles::StaticOptimizationIpoptLinearized::eval_jac_g(
        Ipopt::Index,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index,
        Ipopt::Index *iRow,
        Ipopt::Index *jCol,
        Ipopt::Number *values)
{
    if (new_x){
        dispatch(x);
    }

    if (values == nullptr) {
        // Setup non-zeros values
        Ipopt::Index k(0);
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < m_nMus; ++j){
            for (Ipopt::Index i = 0; i<m; ++i){
                iRow[k] = i;
                jCol[k++] = j;
            }
        }
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < m_nTauResidual; ++j ){
            iRow[k] = j;
            jCol[k++] = j+ static_cast<Ipopt::Index>(m_nMus);
        }
    }
    else
    {
        unsigned int k(0);
        for( unsigned int j = 0; j < m_nMus; j++ )
           {
            for( Ipopt::Index i = 0; i < m; i++ )
            {
                values[k++] = m_jacobian(i,j);
           }
        }
        for( unsigned int j = 0; j < m_nTauResidual; j++ )
            values[k++] = 1;
    }
    return true;
}
