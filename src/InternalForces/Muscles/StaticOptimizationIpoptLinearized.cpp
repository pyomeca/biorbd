#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/StaticOptimizationIpoptLinearized.h"

#include "BiorbdModel.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedTorque.h"
#include "InternalForces/Muscles/State.h"

using namespace BIORBD_NAMESPACE;
using namespace internal_forces;

internal_forces::muscles::StaticOptimizationIpoptLinearized::StaticOptimizationIpoptLinearized(
    Model &model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const rigidbody::GeneralizedTorque &torqueTarget,
    const utils::Vector &activationInit,
    bool useResidual,
    unsigned int pNormFactor,
    int verbose,
    double eps
) :
    internal_forces::muscles::StaticOptimizationIpopt(
        model, Q, Qdot, torqueTarget, activationInit, useResidual,
        pNormFactor, verbose, eps),
    m_jacobian(std::make_shared<utils::Matrix>(*m_nbDof, *m_nbMus))
{
    prepareJacobian();
}


void internal_forces::muscles::StaticOptimizationIpoptLinearized::prepareJacobian()
{
    m_model.updateMuscles(*m_Q, *m_Qdot, true);
    std::vector<std::shared_ptr<internal_forces::muscles::State>> state_zero;
    for (unsigned int i = 0; i<*m_nbMus; ++i) {
        state_zero.push_back(
            std::make_shared<internal_forces::muscles::State>(
                internal_forces::muscles::State(0, 0)));
    }
    const rigidbody::GeneralizedTorque& GeneralizedTorque_zero(
        m_model.muscularJointTorque(state_zero));
    for (unsigned int i = 0; i<*m_nbMus; ++i) {
        std::vector<std::shared_ptr<internal_forces::muscles::State>> state;
        for (unsigned int j = 0; j<*m_nbMus; ++j) {
            unsigned int delta;
            if (j == i) {
                delta = 1;
            } else {
                delta = 0;
            }
            state.push_back(
                std::make_shared<internal_forces::muscles::State>
                (internal_forces::muscles::State(0, delta*1)));
        }
        const rigidbody::GeneralizedTorque& GeneralizedTorque(
            m_model.muscularJointTorque(
                state, *m_Q.get(), *m_Qdot.get()));
        for (unsigned int j = 0; j<*m_nbTorque; ++j) {
            (*m_jacobian)(j, i) =
                GeneralizedTorque(j) - GeneralizedTorque_zero(j);
        }
    }
}

internal_forces::muscles::StaticOptimizationIpoptLinearized::~StaticOptimizationIpoptLinearized()
{

}

bool internal_forces::muscles::StaticOptimizationIpoptLinearized::eval_g(
    Ipopt::Index n,
    const Ipopt::Number *x,
    bool new_x,
    Ipopt::Index m,
    Ipopt::Number *g)
{
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbTorqueResidual);
    assert(static_cast<unsigned int>(m) == *m_nbTorque);
    if (new_x) {
        dispatch(x);
    }

    utils::Vector res(static_cast<unsigned int>(m));
    res.setZero();
    // TODO Optimization using Eigen?
    for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++ ) {
        for (unsigned int j = 0; j<*m_nbMus; j++) {
            res[i] += (*m_jacobian)(i,j)*x[j];
        }
        g[i] = res[i] + (*m_torqueResidual)[i];
    }
    if (*m_verbose >= 2) {
        std::cout << "GeneralizedTorque_musc_approximated = "
                  << res.transpose() << std::endl;
        std::cout << "GeneralizedTorque_residual = "
                  << m_torqueResidual->transpose() << std::endl;
    }
    return true;
}

bool internal_forces::muscles::StaticOptimizationIpoptLinearized::eval_jac_g(
    Ipopt::Index,
    const Ipopt::Number *x,
    bool new_x,
    Ipopt::Index m,
    Ipopt::Index,
    Ipopt::Index *iRow,
    Ipopt::Index *jCol,
    Ipopt::Number *values)
{
    if (new_x) {
        dispatch(x);
    }

    if (values == nullptr) {
        // Setup non-zeros values
        Ipopt::Index k(0);
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbMus; ++j) {
            for (Ipopt::Index i = 0; i<m; ++i) {
                iRow[k] = i;
                jCol[k++] = j;
            }
        }
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbTorqueResidual;
                ++j ) {
            iRow[k] = j;
            jCol[k++] = j+ static_cast<Ipopt::Index>(*m_nbMus);
        }
    } else {
        unsigned int k(0);
        for (unsigned int j = 0; j <* m_nbMus; j++ )
            for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++) {
                values[k++] = (*m_jacobian)(i,j);
            }
        for (unsigned int j = 0; j < *m_nbTorqueResidual; j++) {
            values[k++] = 1;
        }
    }
    return true;
}
