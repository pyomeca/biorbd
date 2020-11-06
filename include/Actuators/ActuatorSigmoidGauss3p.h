#ifndef BIORBD_ACTUATORS_ACTUATOR_SIGMOID_GAUSS_3P_H
#define BIORBD_ACTUATORS_ACTUATOR_SIGMOID_GAUSS_3P_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
class GeneralizedVelocity;
}

namespace actuator {

///
/// \brief Class ActuatorSigmoidGauss3p is a joint actuator type which maximum
/// is 3 parameter gaussian (Gauss3p)
///
class BIORBD_API ActuatorSigmoidGauss3p : public Actuator
{
public:
    ///
    /// \brief Construct Gauss3p actuator
    ///
    ActuatorSigmoidGauss3p();

    ///
    /// \brief Construct Gauss3p actuator from another Gauss3p actuator
    /// \param other The other Gauss3p actuator to copy
    ///
    ActuatorSigmoidGauss3p(
            const biorbd::actuator::ActuatorSigmoidGauss3p& other);

    ///
    /// \brief Construct Gauss3p actuator
    /// \param direction The direction of the Gauss3p actuator (+1 or -1)
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maximal torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    ActuatorSigmoidGauss3p(
            int direction,
            const biorbd::utils::Scalar& theta,
            const biorbd::utils::Scalar& lambda,
            const biorbd::utils::Scalar& offset,
            const biorbd::utils::Scalar& r,
            const biorbd::utils::Scalar& qopt,
            unsigned int dofIdx);

    ///
    /// \brief Construct Gauss3p actuator
    /// \param direction The direction of the Gauss3p actuator
    /// \param Tmax The maximal torque in the eccentric phase
    /// \param T0 The maximal torque isometric
    /// \param wmax Maximum angular velocity above which torque cannot be produced
    /// \param wc Angluar velocity of the vertical asymptote of the concentric hyperbola
    /// \param amin Low plateau level
    /// \param wr 1/10 of the distance amax/amin
    /// \param w1 Mid point plateau
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName Name of the parent joint
    ///
    ActuatorSigmoidGauss3p(
            int direction,
            const biorbd::utils::Scalar& theta,
            const biorbd::utils::Scalar& lambda,
            const biorbd::utils::Scalar& offset,
            const biorbd::utils::Scalar& r,
            const biorbd::utils::Scalar& qopt,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~ActuatorSigmoidGauss3p();

    ///
    /// \brief Deep copy of the Gauss3p actuator
    /// \return A deep copy of the Gauss3p actuator
    ///
    biorbd::actuator::ActuatorSigmoidGauss3p DeepCopy() const;

    /// 
    /// \brief Deep copy of the Gauss3p actuator from another Gauss3p actuator
    /// \param other The Gauss3p actuator to copy
    ///
    void DeepCopy(
            const biorbd::actuator::ActuatorSigmoidGauss3p& other);

    ///
    /// \brief Return the maximal torque (invalid)
    /// \return The maximal torque
    /// torqueMax for ActuatorSigmoidGauss3p must be called with Q and Qdot
    ///
    virtual biorbd::utils::Scalar torqueMax();

    ///
    /// \brief Return the maximal torque at a given Q and Qdot
    /// \param Q The generalized coordinates of the actuator
    /// \param Qdot The generalized velocities of the actuator
    /// \return The maximal torque
    ///
    virtual biorbd::utils::Scalar torqueMax(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            const biorbd::rigidbody::GeneralizedVelocity &Qdot);

protected:
    ///
    /// \brief Set the type of actuator
    ///
    virtual void setType();             

    // Coefficients needed for the Gauss3p
    // For informations on these parameters, see Monique Iris Jackson's these from page 54
    // Torque/angle relationship
    std::shared_ptr<biorbd::utils::Scalar> m_r;         ///< Width of the gaussian curve
    std::shared_ptr<biorbd::utils::Scalar> m_qopt;      ///< Optimal position

    // Coefficients needed for the Sigmoid
    std::shared_ptr<biorbd::utils::Scalar> m_theta;     ///< Amplitude of the Sigmoid
    std::shared_ptr<biorbd::utils::Scalar> m_lambda;    ///< Tilt factor of the Sigmoid
    std::shared_ptr<biorbd::utils::Scalar> m_offset;    ///< Height of the Sigmoid
};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_GAUSS_3P_H
