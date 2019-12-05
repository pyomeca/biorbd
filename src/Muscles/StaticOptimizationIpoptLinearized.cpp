#define BIORBD_API_EXPORTS
#include "Muscles/StaticOptimizationIpoptLinearized.h"

#include "BiorbdModel.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::StaticOptimizationIpoptLinearized::StaticOptimizationIpoptLinearized(
        biorbd::Model &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedTorque &torqueTarget,
        const biorbd::utils::Vector &activationInit,
        bool useResidual,
        unsigned int pNormFactor,
        int verbose,
        double eps
        ) :
    biorbd::muscles::StaticOptimizationIpopt(
        model, Q, Qdot, torqueTarget, activationInit, useResidual,
        pNormFactor, verbose, eps),
    m_jacobian(std::make_shared<biorbd::utils::Matrix>(*m_nbDof, *m_nbMus))
{
    prepareJacobian();
}


void biorbd::muscles::StaticOptimizationIpoptLinearized::prepareJacobian()
{
    m_model.updateMuscles(*m_Q, *m_Qdot, true);
    std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> state_zero;
    for (unsigned int i = 0; i<*m_nbMus; ++i){
        state_zero.push_back(
                    std::make_shared<biorbd::muscles::StateDynamics>(
                        biorbd::muscles::StateDynamics(0, 0)));
    }
    const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque_zero(
                m_model.muscularJointTorque(
                    state_zero, false, m_Q.get(), m_Qdot.get()));
    for (unsigned int i = 0; i<*m_nbMus; ++i) {
        std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> state;
        for (unsigned int j = 0; j<*m_nbMus; ++j) {
            unsigned int delta;
            if (j == i) {
                delta = 1;
            }
            else {
                delta = 0;
            }
            state.push_back(
                        std::make_shared<biorbd::muscles::StateDynamics>
                        (biorbd::muscles::StateDynamics(0, delta*1)));
        }
        const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorque(
                    m_model.muscularJointTorque(
                        state, true, m_Q.get(), m_Qdot.get()));
        for (unsigned int j = 0; j<*m_nbTorque; ++j){
            (*m_jacobian)(j, i) =
                    GeneralizedTorque(j) - GeneralizedTorque_zero(j);
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
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbTorqueResidual);
    assert(static_cast<unsigned int>(m) == *m_nbTorque);
    if (new_x)
        dispatch(x);

    // Compute the torques from muscles
    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque_calcul(
                m_model.muscularJointTorque(
                    *m_states, false, m_Q.get(), m_Qdot.get()));

    biorbd::utils::Vector res(static_cast<unsigned int>(m));
    // TODO Optimization using Eigen?
    for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++ ){  
        for (unsigned int j = 0; j<*m_nbMus; j++)
            res[i] += (*m_jacobian)(i,j)*x[j];
        g[i] = res[i] + (*m_torqueResidual)[i];
    }
    if (*m_verbose >= 2){
        std::cout << "GeneralizedTorque_musc_approximated = "
                  << res.transpose() << std::endl;
        std::cout << "GeneralizedTorque_residual = "
                  << m_torqueResidual->transpose() << std::endl;
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
    if (new_x)
        dispatch(x);

    if (values == nullptr){
        // Setup non-zeros values
        Ipopt::Index k(0);
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbMus; ++j){
            for (Ipopt::Index i = 0; i<m; ++i){
                iRow[k] = i;
                jCol[k++] = j;
            }
        }
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbTorqueResidual; ++j ){
            iRow[k] = j;
            jCol[k++] = j+ static_cast<Ipopt::Index>(*m_nbMus);
        }
    } else {
        unsigned int k(0);
        for (unsigned int j = 0; j <* m_nbMus; j++ )
            for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++)
                values[k++] = (*m_jacobian)(i,j);
        for (unsigned int j = 0; j < *m_nbTorqueResidual; j++)
            values[k++] = 1;
    }
    return true;
}
