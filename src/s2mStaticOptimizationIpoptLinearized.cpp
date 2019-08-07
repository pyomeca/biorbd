#define BIORBD_API_EXPORTS
#include "s2mStaticOptimizationIpoptLinearized.h"

#include "s2mMusculoSkeletalModel.h"
#include "s2mMuscleStateDynamics.h"

s2mStaticOptimizationIpoptLinearized::s2mStaticOptimizationIpoptLinearized(
        s2mMusculoSkeletalModel &model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const s2mVector &activationInit,
        bool useResidual,
        unsigned int pNormFactor,
        int verbose,
        double eps
        ) :
    s2mStaticOptimizationIpopt(model, Q, Qdot, tauTarget, activationInit, useResidual, pNormFactor, verbose, eps),
    m_jacobian(s2mMatrix(m_nDof, m_nMus))
{
    prepareJacobian();
}


void s2mStaticOptimizationIpoptLinearized::prepareJacobian()
{
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    std::vector<s2mMuscleStateDynamics> state_zero;
    for (unsigned int i = 0; i<m_nMus; ++i){
        state_zero.push_back(s2mMuscleStateDynamics(0, 0));
    }
    s2mTau tau_zero = m_model.muscularJointTorque(m_model, state_zero, false, &m_Q, &m_Qdot);
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::vector<s2mMuscleStateDynamics> state;
        for (unsigned int j = 0; j<m_nMus; ++j){
            unsigned int delta;
            if (j == i){
                delta = 1;
            }
            else {
                delta = 0;
            }
            state.push_back(s2mMuscleStateDynamics(0, delta*1));
        }
        s2mTau tau = m_model.muscularJointTorque(m_model, state, true, &m_Q, &m_Qdot);
        for (unsigned int j = 0; j<m_nTau; ++j){
            m_jacobian(j, i) = tau(j)-tau_zero(j);
        }
    }
}

s2mStaticOptimizationIpoptLinearized::~s2mStaticOptimizationIpoptLinearized()
{

}

bool s2mStaticOptimizationIpoptLinearized::eval_g(
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    assert(static_cast<unsigned int>(n) == m_nMus + m_nTauResidual);
    assert(static_cast<unsigned int>(m) == m_nTau);
    if (new_x){
        dispatch(x);
    }
    std::vector<s2mMuscleStateDynamics> state;// controls
    for (unsigned int i = 0; i<m_nMus; ++i){
        state.push_back(s2mMuscleStateDynamics(0, m_activations[i]));
    }
    // Compute the torques from muscles
    s2mTau tau_calcul = m_model.muscularJointTorque(m_model, state, false, &m_Q, &m_Qdot);
    s2mVector res(static_cast<unsigned int>(m));
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

bool s2mStaticOptimizationIpoptLinearized::eval_jac_g(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index nele_jac,
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
