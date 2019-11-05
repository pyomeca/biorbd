#define BIORBD_API_EXPORTS
#include "Muscles/StaticOptimizationIpopt.h"

#include <iostream>
#include <iomanip>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/Vector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::StaticOptimizationIpopt::StaticOptimizationIpopt(
        biorbd::Model &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::rigidbody::GeneralizedTorque &GeneralizedTorqueTarget,
        const biorbd::utils::Vector &activationInit,
        bool useResidual,
        unsigned int pNormFactor,
        int verbose,
        double eps) :
    m_model(model),
    m_nbQ(std::make_shared<unsigned int>(model.nbQ())),
    m_nbQdot(std::make_shared<unsigned int>(model.nbQdot())),
    m_nbMus(std::make_shared<unsigned int>(model.nbMuscleTotal())),
    m_nbDof(std::make_shared<unsigned int>(model.nbDof())),
    m_nbGeneralizedTorque(std::make_shared<unsigned int>(model.nbGeneralizedTorque())),
    m_nbGeneralizedTorqueResidual(std::make_shared<unsigned int>(*m_nbQ)),
    m_eps(std::make_shared<double>(eps)),
    m_activations(std::make_shared<biorbd::utils::Vector>(activationInit)),
    m_Q(std::make_shared<biorbd::rigidbody::GeneralizedCoordinates>(Q)),
    m_Qdot(std::make_shared<biorbd::rigidbody::GeneralizedCoordinates>(Qdot)),
    m_GeneralizedTorqueTarget(std::make_shared<biorbd::rigidbody::GeneralizedTorque>(GeneralizedTorqueTarget)),
    m_GeneralizedTorqueResidual(std::make_shared<biorbd::utils::Vector>(biorbd::utils::Vector::Zero(*m_nbGeneralizedTorque))),
    m_GeneralizedTorquePonderation(std::make_shared<double>(1000)),
    m_states(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>>>(*m_nbMus)),
    m_pNormFactor(std::make_shared<unsigned int>(pNormFactor)),
    m_verbose(std::make_shared<int>(verbose)),
    m_finalSolution(std::make_shared<biorbd::utils::Vector>(biorbd::utils::Vector(*m_nbMus))),
    m_finalResidual(std::make_shared<biorbd::utils::Vector>(biorbd::utils::Vector(*m_nbQ)))
{
    if (*m_eps < 1e-12){
        biorbd::utils::Error::raise("epsilon for partial derivates approximation is too small ! \nLimit for epsilon is 1e-12");
    }
    m_model.updateMuscles(*m_Q, *m_Qdot, true);
    if (!useResidual){
        m_GeneralizedTorqueResidual->setZero();
        *m_nbGeneralizedTorqueResidual = 0;
        *m_GeneralizedTorquePonderation = 0;

    }
}

biorbd::muscles::StaticOptimizationIpopt::~StaticOptimizationIpopt()
{

}

bool biorbd::muscles::StaticOptimizationIpopt::get_nlp_info(
        Ipopt::Index &n,
        Ipopt::Index &m,
        Ipopt::Index &nnz_jac_g,
        Ipopt::Index &nnz_h_lag,
        IndexStyleEnum &index_style)
{
    index_style = TNLP::C_STYLE;

    // variables
    n = static_cast<int>(*m_nbMus + *m_nbGeneralizedTorqueResidual);

    // Constraints
    m = static_cast<int>(*m_nbGeneralizedTorque);
    if (*m_nbGeneralizedTorqueResidual)
        nnz_jac_g = (static_cast<int>(*m_nbMus) + 1) * static_cast<int>(*m_nbGeneralizedTorque);
    else
        nnz_jac_g = static_cast<int>(*m_nbMus) * static_cast<int>(*m_nbGeneralizedTorque);
    nnz_h_lag = static_cast<int>(*m_nbGeneralizedTorque) * static_cast<int>(*m_nbGeneralizedTorque);

    if (*m_verbose >= 2){
        std::cout << "n: " << n << std::endl;
        std::cout << "m: " << m << std::endl;
    }
    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::get_bounds_info(
        Ipopt::Index n,
        Ipopt::Number *x_l,
        Ipopt::Number *x_u,
        Ipopt::Index m,
        Ipopt::Number *g_l,
        Ipopt::Number *g_u)
{
    // Should not be necessary?
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbGeneralizedTorqueResidual);
    assert(static_cast<unsigned int>(m) == *m_nbGeneralizedTorque);

    // Variables
    for( unsigned int i = 0; i < *m_nbMus; ++i ){
        x_l[i] = 0.0001;
        x_u[i] = 0.9999;
    }
    for( unsigned int i = *m_nbMus; i < *m_nbMus+ *m_nbGeneralizedTorqueResidual; ++i ){
        x_l[i] = -1000.0;
        x_u[i] = 1000.0;
    }

    // Constraints
    for( unsigned int i = 0; i < *m_nbGeneralizedTorque; ++i )
        g_l[i] = g_u[i] = 0;

    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::get_starting_point(
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

    for( unsigned int i = 0; i < *m_nbMus; i++ )
        x[i] = (*m_activations)[i];

    for( unsigned int i = 0; i < *m_nbGeneralizedTorqueResidual; i++ )
        x[i+ *m_nbMus] = (*m_GeneralizedTorqueResidual)[i];

    if (*m_verbose >= 2){
        std::cout << std::endl << "Initial guesses" << std::endl;
        std::cout << "Activations = " << m_activations->transpose() << std::endl;
        if (*m_nbGeneralizedTorqueResidual)
            std::cout << "Residuals = " << m_GeneralizedTorqueResidual->transpose() << std::endl;
    }

    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::eval_f(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Number &obj_value)
{
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbGeneralizedTorqueResidual);

    if (new_x)
        dispatch(x);

    // Warning, residual norm HAS to be a non-sqrt, otherwise the problem is degenerated. The activation is just a speed matter.
    obj_value = m_activations->norm(*m_pNormFactor, true) + *m_GeneralizedTorquePonderation*m_GeneralizedTorqueResidual->norm(2, true);

    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::eval_grad_f(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Number *grad_f)
{
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbGeneralizedTorqueResidual);

    if (new_x)
        dispatch(x);

    biorbd::utils::Vector grad_activ(m_activations->normGradient(*m_pNormFactor, true));
    biorbd::utils::Vector grad_residual(m_GeneralizedTorqueResidual->normGradient(2, true));

    for( unsigned i = 0; i < *m_nbMus; i++ )
        grad_f[i] = grad_activ[i];
    for( unsigned int i = 0; i < *m_nbGeneralizedTorqueResidual; i++ )
        grad_f[i+ *m_nbMus] = *m_GeneralizedTorquePonderation*grad_residual[i];
    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::eval_g(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Number *g)
{
    assert(static_cast<unsigned int>(n) == *m_nbMus + *m_nbGeneralizedTorqueResidual);
    assert(static_cast<unsigned int>(m) == *m_nbGeneralizedTorque);
    if (new_x)
        dispatch(x);

    const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueMusc(m_model.muscularJointTorque(*m_states, false, m_Q.get(), m_Qdot.get()));

    // TODO : adjust dimensions for when "root_actuated" is set to false in bioMod file
    for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++ )
         g[i] = GeneralizedTorqueMusc[i] + (*m_GeneralizedTorqueResidual)[i] - (*m_GeneralizedTorqueTarget)[i];

    if (*m_verbose >= 2){
        std::cout << "GeneralizedTorque_musc = " << GeneralizedTorqueMusc.transpose() << std::endl;
        std::cout << "GeneralizedTorque_residual = " << m_GeneralizedTorqueResidual->transpose() << std::endl;
    }
    return true;
}

bool biorbd::muscles::StaticOptimizationIpopt::eval_jac_g(
        Ipopt::Index n,
        const Ipopt::Number *x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Index,
        Ipopt::Index *iRow,
        Ipopt::Index *jCol,
        Ipopt::Number *values)
{
    if (values == nullptr) {
        // Setup non-zeros values
        Ipopt::Index k(0);
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbMus; ++j){
            for (Ipopt::Index i = 0; i<m; ++i){
                iRow[k] = i;
                jCol[k++] = j;
            }
        }
        for (Ipopt::Index j = 0; static_cast<unsigned int>(j) < *m_nbGeneralizedTorqueResidual; ++j ){
            iRow[k] = j;
            jCol[k++] = j+ static_cast<Ipopt::Index>(*m_nbMus);
        }
    } else {
        if (new_x)
            dispatch(x);
        const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueMusc(m_model.muscularJointTorque(*m_states, false, m_Q.get(), m_Qdot.get()));
        unsigned int k(0);
        for( unsigned int j = 0; j < *m_nbMus; ++j ){
            std::vector<std::shared_ptr<biorbd::muscles::StateDynamics>> stateEpsilon;
            for (unsigned int i = 0; i < *m_nbMus; ++i){
                unsigned int delta(0);
                if (i == j)
                    delta = 1;
                stateEpsilon.push_back(std::make_shared<biorbd::muscles::StateDynamics>(biorbd::muscles::StateDynamics(0, (*m_activations)[i]+delta* *m_eps)));
            }
            const biorbd::rigidbody::GeneralizedTorque& GeneralizedTorqueCalculEpsilon(m_model.muscularJointTorque(stateEpsilon, false, m_Q.get(), m_Qdot.get()));
            for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++ ){
                values[k++] = (GeneralizedTorqueCalculEpsilon[i]-GeneralizedTorqueMusc[i])/ *m_eps;
                if (*m_verbose >= 3){
                    std::cout << std::setprecision (20) << std::endl;
                    std::cout << "values[" << k-1 << "]: " << values[k-1] << std::endl;
                    std::cout << "GeneralizedTorqueCalculEpsilon[" << i << "]: " << GeneralizedTorqueCalculEpsilon[i] << std::endl;
                    std::cout << "GeneralizedTorqueMusc[" << i << "]: " << GeneralizedTorqueMusc[i] << std::endl;
                }

            }
        }
        for( unsigned int j = 0; j < *m_nbGeneralizedTorqueResidual; j++ )
            values[k++] = 1;

        if (*m_verbose >= 2){
            k = 0;
            biorbd::utils::Matrix jacobian(biorbd::utils::Matrix::Zero(*m_nbGeneralizedTorque, static_cast<unsigned int>(n)));
            for( unsigned int j = 0; j < *m_nbMus; j++ )
                for( unsigned int i = 0; i < static_cast<unsigned int>(m); i++ )
                    jacobian(i,j) = values[k++];
            for( unsigned int j = 0; j < *m_nbGeneralizedTorqueResidual; j++ ){
                jacobian(j, j+ *m_nbMus) = values[k++];
            }
            std::cout << "jacobian = " << std::endl << jacobian << std::endl;
        }

    }
   return true;
}


void biorbd::muscles::StaticOptimizationIpopt::finalize_solution(
        Ipopt::SolverReturn,
        Ipopt::Index,
        const Ipopt::Number *x,
        const Ipopt::Number*, //z_L,
        const Ipopt::Number*, //z_U,
        Ipopt::Index,
        const Ipopt::Number *,
        const Ipopt::Number *,
        Ipopt::Number obj_value,
        const Ipopt::IpoptData*,
        Ipopt::IpoptCalculatedQuantities*)
{
    // Storing to solution
    dispatch(x);
    m_finalSolution = m_activations;
    m_finalResidual = m_GeneralizedTorqueResidual;

    // Plot it, if it makes sense
    if (*m_verbose >= 1){
        std::cout << std::endl << "Final results" << std::endl;
        std::cout << "f(x*) = " << obj_value << std::endl;
        std::cout << "Activations = " << m_activations->transpose() << std::endl;
        std::cout << "Muscular torques = " << m_model.muscularJointTorque(*m_states, false, m_Q.get(), m_Qdot.get()).transpose() << std::endl;
        std::cout << "GeneralizedTorque target = " << m_GeneralizedTorqueTarget->transpose() << std::endl;
        if (*m_nbGeneralizedTorqueResidual){
            std::cout << "Residual torques= " << m_GeneralizedTorqueResidual->transpose() << std::endl;
        }
//        // Uncomment to show lagrange multipliers
//        std::cout << "Solution of the bound multipliers, z_L and z_U" << std::endl;
//        for( Ipopt::Index i = 0; i < n; i++ )
//            std::cout << "z_L[" << i << "] = " << z_L[i] << ", z_U[" << i << "] = " << z_U[i] << std::endl << std::endl;
    }
}

biorbd::utils::Vector biorbd::muscles::StaticOptimizationIpopt::finalSolution() const{
    return *m_finalSolution;
}

biorbd::utils::Vector biorbd::muscles::StaticOptimizationIpopt::finalResidual() const{
    return *m_finalResidual;
}

void biorbd::muscles::StaticOptimizationIpopt::dispatch(const Ipopt::Number *x)
{
    for(unsigned int i = 0; i < *m_nbMus; i++ ){
        (*m_activations)[i] = x[i];
        (*m_states)[i]->setActivation((*m_activations)[i]);
    }

    for(unsigned int i = 0; i < *m_nbGeneralizedTorqueResidual; i++ )
        (*m_GeneralizedTorqueResidual)[i] = x[i+ *m_nbMus];
}



