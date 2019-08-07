#ifndef S2MINTEGRATOR_H
#define S2MINTEGRATOR_H

#include <vector>
#include <rbdl/Model.h>
#include "biorbdConfig.h"

// The type of container used to hold the state vector
typedef std::vector< double > state_type;

class s2mGenCoord;
class s2mJoints;
class BIORBD_API s2mIntegrator
{
    public:
        s2mIntegrator();

        void integrate(RigidBodyDynamics::Model*, const s2mGenCoord&, const Eigen::VectorXd&, const double&, const double&, const double&);
        void operator() ( const state_type &x , state_type &dxdt , const double /* t */ );

        s2mGenCoord getX(const unsigned int&); // Return the Q for a given step
        void showAll(); // Show every steps with every dof
        unsigned int steps() const {return m_steps+1;}
    protected:
        unsigned int m_nbre; // Nombre d'élément dans l'intégration
        unsigned int m_steps; // Nombre de step pour l'intégration
        RigidBodyDynamics::Model * m_model; // Model dans lequel il faut appeler forwardDynamics

        // Déclarer un observeur
        std::vector<state_type> m_x_vec;
        std::vector<double> m_times;
        Eigen::VectorXd m_u; // Effecteurs


        // Structure permettant de conserver les valeurs
        struct push_back_state_and_time{
            std::vector< state_type >& m_states;
            std::vector< double >& m_times;
            push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
            : m_states( states ) , m_times( times ) { }
            void operator()( const state_type &x , double t ){
                m_states.push_back( x );
                m_times.push_back( t );
            }
        };

};

#endif // S2MINTEGRATOR_H
