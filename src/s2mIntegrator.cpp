#define BIORBD_API_EXPORTS
#include "s2mIntegrator.h"

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <rbdl/Dynamics.h>
#include "s2mJoints.h"
#include "s2mError.h"
#include "s2mString.h"
#include "s2mGenCoord.h"

s2mIntegrator::s2mIntegrator(){

}

void s2mIntegrator::operator() ( const state_type &x , state_type &dxdt , const double ){
    // Équation différentielle : x/xdot => xdot/xddot
    s2mGenCoord Q(m_nbre);
    s2mGenCoord QDot(m_nbre);
    s2mGenCoord QDDot(Eigen::VectorXd::Zero(m_nbre));
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

void s2mIntegrator::showAll(){
    std::cout << "Test:" << std::endl;
    for (unsigned int i=0; i<=m_steps; i++){
        std::cout << m_times[i];
        for (unsigned int j=0; j<m_nbre; j++)
            std::cout << " " << m_x_vec[i][j];
        std::cout << std::endl;
    }
}

s2mGenCoord s2mIntegrator::getX(const unsigned int &idx){
    s2mGenCoord out(m_nbre*2);
    s2mError::s2mAssert(idx <= m_steps, "Trying to get Q outside range");
    for (unsigned int i=0; i<m_nbre*2; i++){
        out(i) = m_x_vec[idx][i];
        }
    return out;
}

void s2mIntegrator::integrate(RigidBodyDynamics::Model *model, const s2mGenCoord &v, const Eigen::VectorXd& u, const double &t0, const double &tEnd, const double &time_step){
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
