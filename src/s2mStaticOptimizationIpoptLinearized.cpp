#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimizationIpoptLinearized.h"


s2mStaticOptimizationIpoptLinearized::s2mStaticOptimizationIpoptLinearized(
        s2mMusculoSkeletalModel &model,
        const s2mGenCoord &Q,
        const s2mGenCoord &Qdot,
        const s2mTau &tauTarget,
        const s2mVector &activationInit,
        bool useResidual,
        unsigned int p,
        int verbose,
        double eps
        ) :
    s2mStaticOptimizationIpopt(model, Q, Qdot, tauTarget, activationInit, useResidual, verbose, p, eps),
    m_jacobian(s2mMatrix(m_nDof, m_nMus))
{
    prepareJacobian();
}


void s2mStaticOptimizationIpoptLinearized::prepareJacobian()
{
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    std::vector<s2mMuscleStateActual> state_zero;
    for (unsigned int i = 0; i<m_nMus; ++i){
        state_zero.push_back(s2mMuscleStateActual(0, 0));
    }
    s2mTau tau_zero = m_model.muscularJointTorque(m_model, state_zero, false, &m_Q, &m_Qdot);
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::vector<s2mMuscleStateActual> state;
        for (unsigned int j = 0; j<m_nMus; ++j){
            unsigned int delta;
            if (j == i){
                delta = 1;
            }
            else {
                delta = 0;
            }
            state.push_back(s2mMuscleStateActual(0, delta*1));
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
    if (new_x){
        dispatch(x);
    }
    std::vector<s2mMuscleStateActual> state;// controls
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::cout << "m_activation[" << i << "]: " << m_activations[i] << std::endl;
        state.push_back(s2mMuscleStateActual(0, m_activations[i]));
    }
    // Compute the torques from muscles
    s2mTau tau_calcul = m_model.muscularJointTorque(m_model, state, false, &m_Q, &m_Qdot);

    for( Ipopt::Index i = 0; i < m; i++ )
       {
        double res(0);
        for (unsigned int j = 0; static_cast<int>(j)<n; j++){
            res += m_jacobian(i,j)*x[j];
        }
        g[i] = res;
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
        unsigned int k(0);
    // return the structure of the Jacobian
    // this particular Jacobian is dense
        for (unsigned int j = 0; j<m_nMus; ++j){
            for (unsigned int i = 0; i<m_nTau; ++i){
                iRow[k] = static_cast<int>(i);
                jCol[k] = static_cast<int>(j);
                ++k;
            }
        }

    }
    else
    {
        unsigned int k(0);
        for( Ipopt::Index j = 0; j < n; j++ )
           {

            for( Ipopt::Index i = 0; i < m; i++ )
            {
                values[k] = m_jacobian(i,j);
                ++k;
           }
        }
    }
    return true;
}
