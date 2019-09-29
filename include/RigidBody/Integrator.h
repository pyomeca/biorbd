#ifndef BIORBD_UTILS_INTEGRATOR_H
#define BIORBD_UTILS_INTEGRATOR_H

#include <memory>
#include <vector>
#include <rbdl/Model.h>
#include "biorbdConfig.h"

// The type of container used to hold the state vector
typedef std::vector< double > state_type;

namespace biorbd {
namespace utils {
class Vector;
}

namespace rigidbody {
class Joints;
class GeneralizedCoordinates;

class BIORBD_API Integrator
{
public:
    Integrator();
    biorbd::rigidbody::Integrator DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Integrator& other);

    void integrate(
            biorbd::rigidbody::Joints &model,
            const biorbd::utils::Vector& Q_Qdot,
            const biorbd::utils::Vector& u,
            double t0,
            double tend,
            double timeStep);
    void operator() (
            const state_type &x,
            state_type &dxdt,
            double t );

    biorbd::utils::Vector getX(
            unsigned int idx); // Return the Q for a given step
    void showAll(); // Show every steps with every dof
    unsigned int steps() const;
protected:
    std::shared_ptr<unsigned int> m_nbre; // Nombre d'élément dans l'intégration
    std::shared_ptr<unsigned int> m_steps; // Nombre de step pour l'intégration
    std::shared_ptr<RigidBodyDynamics::Model> m_model; // Model dans lequel il faut appeler forwardDynamics

    // Déclarer un observeur
    std::shared_ptr<std::vector<state_type>> m_x_vec;
    std::shared_ptr<std::vector<double>> m_times;
    std::shared_ptr<biorbd::utils::Vector> m_u; // Effecteurs


    // Structure permettant de conserver les valeurs
    struct push_back_state_and_time{
        std::vector< state_type >& m_states;
        std::vector< double >& m_times;
        push_back_state_and_time(
                std::vector< state_type > &states ,
                std::vector< double > &times )
        : m_states( states ) , m_times( times ) { }
        void operator()(
                const state_type &x ,
                double t ){
            m_states.push_back( x );
            m_times.push_back( t );
        }
    };

};

}}

#endif // BIORBD_UTILS_INTEGRATOR_H
