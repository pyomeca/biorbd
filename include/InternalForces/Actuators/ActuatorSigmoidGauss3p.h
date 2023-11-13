#ifndef BIORBD_ACTUATORS_ACTUATOR_SIGMOID_GAUSS_3P_H
#define BIORBD_ACTUATORS_ACTUATOR_SIGMOID_GAUSS_3P_H

#include "biorbdConfig.h"
#include "InternalForces/Actuators/Actuator.h"

namespace BIORBD_NAMESPACE
{
namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
}

namespace internal_forces
{
namespace actuator
{

///
/// \brief Class ActuatorSigmoidGauss3p is a joint actuator type which maximum
/// is 3 parameter gaussian (Gauss3p)
/// Please note that all parameters are given in degrees
///
class BIORBD_API ActuatorSigmoidGauss3p : public Actuator
{
public:
    ///
    /// \brief Construct SigmoidGauss3p actuator
    ///
    ActuatorSigmoidGauss3p();

    ///
    /// \brief Construct SigmoidGauss3p actuator from another SigmoidGauss3p actuator
    /// \param other The other SigmoidGauss3p actuator to copy
    ///
    ActuatorSigmoidGauss3p(
        const ActuatorSigmoidGauss3p& other);

    ///
    /// \brief Construct SigmoidGauss3p actuator
    /// \param direction The direction of the SigmoidGauss3p actuator (+1 or -1)
    /// \param theta Amplitude of the sigmoid
    /// \param lambda Tilt factor of the sigmoid
    /// \param offset Height of the sigmoid
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    ActuatorSigmoidGauss3p(
        int direction,
        const utils::Scalar& theta,
        const utils::Scalar& lambda,
        const utils::Scalar& offset,
        const utils::Scalar& r,
        const utils::Scalar& qopt,
        size_t dofIdx);

    ///
    /// \brief Construct Gauss3p actuator
    /// \param direction The direction of the Gauss3p actuator
    /// \param theta Amplitude of the sigmoid
    /// \param lambda Tilt factor of the sigmoid
    /// \param offset Height of the sigmoid
    /// \param r width of the gaussian curve
    /// \param qopt Optimal position
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName Name of the parent joint
    ///
    ActuatorSigmoidGauss3p(
        int direction,
        const utils::Scalar& theta,
        const utils::Scalar& lambda,
        const utils::Scalar& offset,
        const utils::Scalar& r,
        const utils::Scalar& qopt,
        size_t dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~ActuatorSigmoidGauss3p();

    ///
    /// \brief Deep copy of the SigmoidGauss3p actuator
    /// \return A deep copy of the SigmoidGauss3p actuator
    ///
    ActuatorSigmoidGauss3p DeepCopy() const;

    ///
    /// \brief Deep copy of the SigmoidGauss3p actuator from another SigmoidGauss3p actuator
    /// \param other The SigmoidGauss3p actuator to copy
    ///
    void DeepCopy(
        const ActuatorSigmoidGauss3p& other);

    ///
    /// \brief Return the maximal torque (invalid)
    /// \return The maximal torque
    /// torqueMax for ActuatorSigmoidGauss3p must be called with Q and Qdot
    ///
    virtual utils::Scalar torqueMax();

    ///
    /// \brief Return the maximal torque at a given Q and Qdot
    /// \param Q The generalized coordinates of the actuator
    /// \param Qdot The generalized velocities of the actuator
    /// \return The maximal torque
    ///
    virtual utils::Scalar torqueMax(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot);

protected:
    ///
    /// \brief Set the type of actuator
    ///
    virtual void setType();

    // Coefficients needed for the SigmoidGauss3p
    // Torque/angle relationship for the Gauss3p
    std::shared_ptr<utils::Scalar>
    m_r;         ///< Width of the gaussian curve
    std::shared_ptr<utils::Scalar> m_qopt;      ///< Optimal position

    // Coefficients needed for the Sigmoid
    std::shared_ptr<utils::Scalar>
    m_theta;     ///< Amplitude of the Sigmoid
    std::shared_ptr<utils::Scalar>
    m_lambda;    ///< Tilt factor of the Sigmoid
    std::shared_ptr<utils::Scalar> m_offset;    ///< Height of the Sigmoid
};

}
}
}

#endif // BIORBD_ACTUATORS_ACTUATOR_SIGMOID_GAUSS_3P_H
