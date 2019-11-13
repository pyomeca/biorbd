#define BIORBD_API_EXPORTS
#include "RigidBody/Integrator.h"

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <rbdl/Dynamics.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"

biorbd::rigidbody::Integrator::Integrator() :
    m_nbre(std::shared_ptr<unsigned int>()),
    m_steps(std::shared_ptr<unsigned int>()),
    m_model(std::shared_ptr<RigidBodyDynamics::Model>()),
    m_x_vec(std::shared_ptr<std::vector<state_type>>()),
    m_times(std::shared_ptr<std::vector<double>>()),
    m_u(std::shared_ptr<biorbd::utils::Vector>())

{

}

biorbd::rigidbody::Integrator biorbd::rigidbody::Integrator::DeepCopy() const
{
    biorbd::rigidbody::Integrator copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Integrator::DeepCopy(const biorbd::rigidbody::Integrator &other)
{
    *m_nbre = *other.m_nbre;
    *m_steps = *other.m_steps;
    *m_model = *other.m_model;
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
    biorbd::rigidbody::GeneralizedCoordinates Q(*m_nbre);
    biorbd::rigidbody::GeneralizedCoordinates QDot(*m_nbre);
    biorbd::rigidbody::GeneralizedCoordinates QDDot(biorbd::utils::Vector(*m_nbre).setZero());
    for (unsigned int i=0; i<*m_nbre; i++){
        Q(i) = x[i];
        QDot(i) = x[i+*m_nbre];
    }

    RigidBodyDynamics::ForwardDynamics (*m_model, Q, QDot, *m_u, QDDot);

    // Faire sortir xdot/xddot
    for (unsigned int i=0; i<*m_nbre; i++){
        dxdt[i] = QDot[i];
        dxdt[i + *m_nbre] = QDDot[i];
    }

}

void biorbd::rigidbody::Integrator::showAll(){
    std::cout << "Test:" << std::endl;
    for (unsigned int i=0; i<=*m_steps; i++){
        std::cout << (*m_times)[i];
        for (unsigned int j=0; j<*m_nbre; j++)
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
    biorbd::utils::Vector out(*m_nbre*2);
    biorbd::utils::Error::check(idx <= *m_steps, "Trying to get Q outside range");
    for (unsigned int i=0; i<*m_nbre*2; i++){
        out(i) = (*m_x_vec)[idx][i];
        }
    return out;
}

void biorbd::rigidbody::Integrator::integrate(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::Vector &Q_Qdot,
        const biorbd::utils::Vector &u,
        double t0,
        double tend,
        double timeStep){
    // Stocker le nombre d'élément à traiter
    *m_nbre = static_cast<unsigned int>(Q_Qdot.rows())/2; // Q et Qdot
    *m_u = u; // Copier les effecteurs
    *m_model = model;

    // Remplissage de la variable par les positions et vitesse
    state_type x(*m_nbre*2);
    for (unsigned int i=0; i<*m_nbre*2; i++)
        x[i] = Q_Qdot(i);

    // Choix de l'algorithme et intégration
    boost::numeric::odeint::runge_kutta4< state_type > stepper;
    *m_steps = static_cast<unsigned int>(
                boost::numeric::odeint::integrate_const(
                    stepper, (*this), x, t0, tend, timeStep,
                    push_back_state_and_time( *m_x_vec , *m_times )));
}
