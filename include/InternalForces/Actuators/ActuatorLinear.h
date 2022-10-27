#ifndef BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
#define BIORBD_ACTUATORS_ACTUATOR_LINEAR_H

#include "biorbdConfig.h"
#include "InternalForces/Actuators/Actuator.h"

namespace BIORBD_NAMESPACE
{
namespace rigidbody
{
class GeneralizedCoordinates;
}

namespace internal_forces
{
namespace actuator
{

///
/// \brief Class ActuatorLinear is a joint actuator type that linearly evolves
///
class BIORBD_API ActuatorLinear : public Actuator
{
public:
    ///
    /// \brief Construct a linear actuator
    ///
    ActuatorLinear();

    ///
    /// \brief Construct a linear actuator from another linear actuator
    /// \param other The other linear actuator
    ///
    ActuatorLinear(
        const ActuatorLinear& other);

    ///
    /// \brief Construct a linear actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param T0 The maximal torque isometric
    /// \param slope The slope
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    ActuatorLinear(
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

    ActuatorLinear(
        int direction,
        const utils::Scalar& T0,
        const utils::Scalar& slope,
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~ActuatorLinear();

    ///
    /// \brief Deep copy of the linear actuator
    /// \return A deep copy of the linear actuator
    ///
    ActuatorLinear DeepCopy() const;

    ///
    /// \brief Deep copy of the linear actuator from another linear actuator
    /// \param other The linear actuator to copy
    ///
    void DeepCopy(
        const ActuatorLinear& other);

    ///
    /// \brief Return the maximal torque (invalid)
    /// \return The maximal torque
    /// torqueMax for ActuatorLinear must be called with Q
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

#endif // BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
