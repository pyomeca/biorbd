#define BIORBD_API_EXPORTS
#include "../include/s2mStaticOptimizationIpOptLinearized.h"


s2mStaticOptimizationIpoptLinearized::s2mStaticOptimizationIpoptLinearized(s2mMusculoSkeletalModel &model,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mTau& tau_init,
        const s2mVector &activationInit,
        unsigned int p,
        const double epsilon
        ) :
    s2mStaticOptimizationIpopt(model, Q, Qdot, tau_init, activationInit, p, epsilon),
    m_jacobian(s2mMatrix(m_nDof, m_nMus))
{
    prepareJacobian();
}


s2mStaticOptimizationIpoptLinearized::s2mStaticOptimizationIpoptLinearized(
        s2mMusculoSkeletalModel &model,
        const s2mGenCoord& Q, // states
        const s2mGenCoord& Qdot, // derived states
        const s2mTau& tau_init,
        unsigned int p,
        const double epsilon
        ) :
    s2mStaticOptimizationIpopt(model, Q, Qdot, tau_init, p, epsilon),
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
    s2mTau tau_zero = m_model.muscularJointTorque(m_model, state_zero, true, &m_Q, &m_Qdot);
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
            state_zero.push_back(s2mMuscleStateActual(0, delta*1));
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
        fillActivation(n, x);
    }
    std::vector<s2mMuscleStateActual> state;// controls
    for (unsigned int i = 0; i<m_nMus; ++i){
        std::cout << "m_activation[" << i << "]: " << m_activation[i] << std::endl;
        state.push_back(s2mMuscleStateActual(0, m_activation[i]));
    }
    // Compute the torques from muscles
    m_model.updateMuscles(m_model, m_Q, m_Qdot, true);
    s2mTau tau_calcul = m_model.muscularJointTorque(m_model, state, true, &m_Q, &m_Qdot);

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
        Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    if (new_x){
        fillActivation(n, x);
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
