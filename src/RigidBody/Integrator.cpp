#define BIORBD_API_EXPORTS
#include "RigidBody/Integrator.h"

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <rbdl/Dynamics.h>

#include "Utils/Error.h"
#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"

biorbd::rigidbody::Integrator::Integrator(biorbd::rigidbody::Joints &model) :
    m_steps(std::make_shared<unsigned int>()),
    m_model(&model),
    m_x_vec(std::make_shared<std::vector<state_type>>()),
    m_times(std::make_shared<std::vector<double>>()),
    m_u(std::make_shared<biorbd::utils::Vector>()) {

}

biorbd::rigidbody::Integrator biorbd::rigidbody::Integrator::DeepCopy() const
{
    biorbd::rigidbody::Integrator copy(*this->m_model);
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Integrator::DeepCopy(const biorbd::rigidbody::Integrator &other)
{
    *m_steps = *other.m_steps;
    m_model = other.m_model;
    m_x_vec->resize(other.m_x_vec->size());
    for (unsigned int i=0; i<other.m_x_vec->size(); ++i)
        (*m_x_vec)[i] = (*other.m_x_vec)[i];
    m_times->resize(other.m_times->size());
    for (unsigned int i=0; i<other.m_times->size(); ++i)
        (*m_times)[i] = (*other.m_times)[i];
    *m_u = *other.m_u;
}

void biorbd::rigidbody::Integrator::operator() (
        const state_type &x ,
        state_type &dxdt ,
        double ){

    // Équation différentielle : x/xdot => xdot/xddot
    biorbd::rigidbody::GeneralizedCoordinates Q(*m_model);
    biorbd::rigidbody::GeneralizedCoordinates QDot(*m_model);
    biorbd::rigidbody::GeneralizedCoordinates QDDot(*m_model);
    QDDot.setZero();
    for (unsigned int i=0; i<*m_nQ; i++){
        Q(i) = x[i];
    }
    for (unsigned int i=0; i<*m_nQdot; i++){
        QDot(i) = x[i+*m_nQ];
    }

    RigidBodyDynamics::ForwardDynamics (*m_model, Q, QDot, *m_u, QDDot);

    // Faire sortir xdot/xddot
    for (unsigned int i=0; i<*m_nQ; i++){
        dxdt[i] = QDot[i];
    }
    for (unsigned int i=0; i<*m_nQdot; i++){
        dxdt[i + *m_nQ] = QDDot[i];
    }

}

void biorbd::rigidbody::Integrator::showAll(){
    std::cout << "Test:" << std::endl;
    for (unsigned int i=0; i <= *m_steps; i++){
        std::cout << (*m_times)[i];
        for (unsigned int j = 0; j < *m_nQ + *m_nQdot; j++)
            std::cout << " " << (*m_x_vec)[i][j];
        std::cout << std::endl;
    }
}

unsigned int biorbd::rigidbody::Integrator::steps() const
{
    return *m_steps+1;
}

biorbd::utils::Vector biorbd::rigidbody::Integrator::getX(
        unsigned int idx){
    biorbd::utils::Vector out(*m_nQ + *m_nQdot);
    biorbd::utils::Error::check(idx <= *m_steps, "Trying to get Q outside range");
    for (unsigned int i=0; i<*m_nQ + *m_nQdot; i++){
        out(i) = (*m_x_vec)[idx][i];
        }
    return out;
}

double biorbd::rigidbody::Integrator::time(unsigned int idx)
{
    return (*m_times)[idx];
}

void biorbd::rigidbody::Integrator::integrate(
        const biorbd::utils::Vector &Q_Qdot,
        const biorbd::utils::Vector &u,
        double t0,
        double tend,
        double timeStep){
    // These variable can't be computer a construct time because of
    // interaction calls with biorbd::rigidbody::Joints
    m_nQ = std::make_shared<unsigned int>(m_model->nbQ());
    m_nQdot = std::make_shared<unsigned int>(m_model->nbQdot());

    // Assume constant torque over the whole integration
    *m_u = u;

    // Remplissage de la variable par les positions et vitesse
    state_type x(*m_nQ + *m_nQdot);
    for (unsigned int i=0; i<*m_nQ + *m_nQdot; i++)
        x[i] = Q_Qdot(i);

    launchIntegrate(x, t0, tend, timeStep);
}

void biorbd::rigidbody::Integrator::launchIntegrate(
        state_type& x,
        double t0,
        double tend,
        double timeStep)
{
    // Choix de l'algorithme et intégration
    boost::numeric::odeint::runge_kutta4< state_type > stepper;
    *m_steps = static_cast<unsigned int>(
                boost::numeric::odeint::integrate_const(
                    stepper, *this, x, t0, tend, timeStep,
                    push_back_state_and_time( *m_x_vec , *m_times )));
}
