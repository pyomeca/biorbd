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
///
/// \brief Namespace rigidbody containing the class Joints and GeneralizedCoordinates
namespace rigidbody {
class Joints;
class GeneralizedCoordinates;

///
/// \brief Class Integrator
///
class BIORBD_API Integrator
{
public:

    ///
    /// \brief Construct integrator
    ///
    Integrator();

    ///
    /// \brief Deep copy of integrator
    /// \return Copy of integrator
    ///
    biorbd::rigidbody::Integrator DeepCopy() const;

    ///
    /// \brief Deep copy of integrator in another object
    /// \param other Integrator to copy
    ///
    void DeepCopy(const biorbd::rigidbody::Integrator& other);

    ///
    /// \brief Perform integration
    /// \param model The model to integrate
    /// \param Q_Qdot Vector containing position and velocity
    /// \param u Effectors
    /// \param t0 The initial time
    /// \param tend The final integration time
    /// \param timeStep Time step for the integration
    ///
    void integrate(
            biorbd::rigidbody::Joints &model,
            const biorbd::utils::Vector& Q_Qdot,
            const biorbd::utils::Vector& u,
            double t0,
            double tend,
            double timeStep);

    ///
    /// \brief Construct operator
    /// \param x Generalized coordinates: Q
    /// \param dxdt Derivate of x with respect to t
    /// \param t Time
    ///
    void operator() (
            const state_type &x,
            state_type &dxdt,
            double t );

    ///
    /// \brief Return the Q for a given step
    /// \param idx Index of the step
    /// \return Q for a given step
    ///

    biorbd::utils::Vector getX(
            unsigned int idx);

    ///
    /// \brief Show every steps with every DoF
    /// 
    void showAll();

    ///
    /// \brief Return the number of steps
    /// \return The number of steps
    ///
    unsigned int steps() const;

protected:
    std::shared_ptr<unsigned int> m_nbre; ///< Number of elements in the integration
    std::shared_ptr<unsigned int> m_steps; ///< Number of steps in the integration
    std::shared_ptr<RigidBodyDynamics::Model> m_model; ///< Model in which we will call forwardDynamics

    // Declare an observer
    std::shared_ptr<std::vector<state_type>> m_x_vec; ///< Vector of x
    std::shared_ptr<std::vector<double>> m_times; ///< Vector of time
    std::shared_ptr<biorbd::utils::Vector> m_u; ///< Effectors



    ///
    /// \brief Structure containing the states and time
    ///
    struct push_back_state_and_time{
        std::vector< state_type >& m_states; ///< The states
        std::vector< double >& m_times; ///< The times

        ///
        /// \brief Store the states and times
        /// \param states A vector containing the states
        /// \param times A vector containing the times
        ///
        push_back_state_and_time(
                std::vector< state_type > &states ,
                std::vector< double > &times )
        : m_states( states ) , m_times( times ) { }

        ///
        /// \brief TODO:
        /// \param x
        /// \param t
        ///
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
