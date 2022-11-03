#ifndef BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_LINEAR_H
#define BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_LINEAR_H

#include "biorbdConfig.h"
#include "InternalForces/PassiveTorques/PassiveTorque.h"

namespace BIORBD_NAMESPACE
{
namespace rigidbody
{
class GeneralizedCoordinates;
}

namespace internal_forces
{
namespace passive_torques
{

///
/// \brief Class PassiveTorqueLinear is a joint passive torque type that linearly evolves
///
class BIORBD_API PassiveTorqueLinear : public PassiveTorque
{
public:
    ///
    /// \brief Construct a linear actuator
    ///
    PassiveTorqueLinear();

    ///
    /// \brief Construct a linear actuator from another linear actuator
    /// \param other The other linear actuator
    ///
    PassiveTorqueLinear(
        const PassiveTorqueLinear& other);

    ///
    /// \brief Construct a linear actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param T0 The maximal torque isometric
    /// \param slope The slope
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    PassiveTorqueLinear(
        int direction,
        const utils::Scalar& T0,
        const utils::Scalar& slope,
        unsigned int dofIdx);

    ///
    /// \brief Construct a linear actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param T0  The maximal torque isometric
    /// \param slope The slope
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName The name of the parent joint
    ///

    PassiveTorqueLinear(
        int direction,
        const utils::Scalar& T0,
        const utils::Scalar& slope,
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~PassiveTorqueLinear();

    ///
    /// \brief Deep copy of the linear actuator
    /// \return A deep copy of the linear actuator
    ///
    PassiveTorqueLinear DeepCopy() const;

    ///
    /// \brief Deep copy of the linear actuator from another linear actuator
    /// \param other The linear actuator to copy
    ///
    void DeepCopy(
        const PassiveTorqueLinear& other);

    ///
    /// \brief Return the maximal torque (invalid)
    /// \return The maximal torque
    /// torqueMax for PassiveTorqueLinear must be called with Q
    ///
    virtual utils::Scalar torqueMax();

    ///
    /// \brief Return the maximal torque at a given Q
    /// \param Q The generalized coordinates of the actuator
    /// \return The maximal torque
    ///
    virtual utils::Scalar torqueMax(
        const rigidbody::GeneralizedCoordinates &Q) const;

protected:

    ///
    /// \brief Set the type of actuator
    ///
    virtual void setType();

    // mx+b
    std::shared_ptr<utils::Scalar> m_m; ///< Slope
    std::shared_ptr<utils::Scalar> m_b; ///< Torque at zero

};

}
}
}

#endif // BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_H
