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

///
/// \brief Allow for integration of the kinematics from the Forward dynamics
///
class BIORBD_API Integrator
{
public:

    ///
    /// \brief Construct an integrator
    ///
    Integrator(biorbd::rigidbody::Joints &model);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Integrator();

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
    /// \brief Perform the integration from t0 to tend using a RK4
    /// \param Q_Qdot Vector containing the initial generalized coordinates and velocities
    /// \param u Effectors to be used in the forward dynamics, it is assumed to be constant over all the integration
    /// \param t0 The initial time
    /// \param tend The final integration time
    /// \param timeStep Time step of the integration
    ///
    void integrate(
            const biorbd::utils::Vector& Q_Qdot,
            const biorbd::utils::Vector& u,
            double t0,
            double tend,
            double timeStep);

    ///
    /// \brief The right-hand side function
    /// \param x The generalized coordinate and velocities
    /// \param dxdt The time derivative of x
    /// \param t The time at which it is performed
    ///
    virtual void operator() (
            const state_type &x,
            state_type &dxdt,
            double t );

    ///
    /// \brief Return the Q and Qdot of an already performed integration at a given index
    /// \param idx The index of the step
    /// \return The solution at index idx
    ///
    biorbd::utils::Vector getX(
            unsigned int idx);

    ///
    /// \brief Return the actual time of an already performed integration at step index
    /// \param idx The index of the step
    /// \return The time at step index
    ///
    double time(unsigned int idx); 

    ///
    /// \brief Print the solution in std::cout
    /// 
    void showAll();

    ///
    /// \brief Return the number of steps performed
    /// \return The number of steps
    ///
    unsigned int steps() const;

protected:
    std::shared_ptr<unsigned int> m_steps; ///< Number of steps in the integration
    biorbd::rigidbody::Joints* m_model; ///< Model in which Model we have to call forwardDynamics
    std::shared_ptr<unsigned int> m_nQ; ///< Number of general coordinates
    std::shared_ptr<unsigned int> m_nQdot;///< Number of general velocities

    // Declare an observer
    std::shared_ptr<std::vector<state_type>> m_x_vec; ///< Vector of x
    std::shared_ptr<std::vector<double>> m_times; ///< Vector of time
    std::shared_ptr<biorbd::utils::Vector> m_u; ///< Effectors

    ///
    /// \brief Launch integration
    /// \param x Initial state
    /// \param t0 Start time
    /// \param tend End time
    /// \param timeStep Time step (dt)
    ///
    virtual void launchIntegrate(
            state_type& x,
            double t0,
            double tend,
            double timeStep);

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
        /// \brief Append a state to the states
        /// \param x The state to append
        /// \param t The time
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
