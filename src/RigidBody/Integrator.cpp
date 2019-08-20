#define BIORBD_API_EXPORTS
#include "RigidBody/Integrator.h"

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <rbdl/Dynamics.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"

biorbd::rigidbody::Integrator::Integrator(){

}

void biorbd::rigidbody::Integrator::operator() (
        const state_type &x ,
        state_type &dxdt ,
        double ){
    // Équation différentielle : x/xdot => xdot/xddot
    biorbd::rigidbody::GeneralizedCoordinates Q(m_nbre);
    biorbd::rigidbody::GeneralizedCoordinates QDot(m_nbre);
    biorbd::rigidbody::GeneralizedCoordinates QDDot(biorbd::utils::Vector(m_nbre).setZero());
    for (unsigned int i=0; i<m_nbre; i++){
        Q(i) = x[i];
        QDot(i) = x[i+m_nbre];
    }

    RigidBodyDynamics::ForwardDynamics (*m_model, Q, QDot, m_u, QDDot);

    // Faire sortir xdot/xddot
    for (unsigned int i=0; i<m_nbre; i++){
        dxdt[i] = QDot[i];
        dxdt[i+m_nbre] = QDDot[i];
    }

}

void biorbd::rigidbody::Integrator::showAll(){
    std::cout << "Test:" << std::endl;
    for (unsigned int i=0; i<=m_steps; i++){
        std::cout << m_times[i];
        for (unsigned int j=0; j<m_nbre; j++)
            std::cout << " " << m_x_vec[i][j];
        std::cout << std::endl;
    }
}

biorbd::rigidbody::GeneralizedCoordinates biorbd::rigidbody::Integrator::getX(unsigned int idx){
    biorbd::rigidbody::GeneralizedCoordinates out(m_nbre*2);
    biorbd::utils::Error::error(idx <= m_steps, "Trying to get Q outside range");
    for (unsigned int i=0; i<m_nbre*2; i++){
        out(i) = m_x_vec[idx][i];
        }
    return out;
}

void biorbd::rigidbody::Integrator::integrate(
        biorbd::rigidbody::Joints *model,
        const biorbd::rigidbody::GeneralizedCoordinates &v,
        const biorbd::utils::Vector &u,
        double t0,
        double tEnd,
        double time_step){
    // Stocker le nombre d'élément à traiter
    m_nbre = static_cast<unsigned int>(v.rows())/2; // Q et Qdot
    m_u = u; // Copier les effecteurs
    m_model = model;

    // Remplissage de la variable par les positions et vitesse
    state_type x(m_nbre*2);
    for (unsigned int i=0; i<m_nbre*2; i++)
        x[i] = v(i);

    // Choix de l'algorithme et intégration
    boost::numeric::odeint::runge_kutta4< state_type > stepper;
    m_steps = static_cast<unsigned int>(boost::numeric::odeint::integrate_const( stepper, (*this), x, t0, tEnd, time_step, push_back_state_and_time( m_x_vec , m_times )));
}
