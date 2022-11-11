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
    /// \brief Construct a linear passive torque
    ///
    PassiveTorqueLinear();

    ///
    /// \brief Construct an idealized passive torque from another muscle
    /// \param other The other muscle
    ///
    PassiveTorqueLinear(
        const PassiveTorqueLinear& other);

    ///
    /// \brief Construct a linear passive torque
    /// \param T0 The maximal torque isometric
    /// \param slope The slope
    /// \param dofIdx Index of the DoF associated with passive torque
    ///
    PassiveTorqueLinear(
        const utils::Scalar& T0,
        const utils::Scalar& slope,
        unsigned int dofIdx);

    ///
    /// \brief Construct a linear passive torque
    /// \param T0  The maximal torque isometric
    /// \param slope The slope
    /// \param dofIdx Index of the DoF associated with passive torque
    /// \param jointName The name of the parent joint
    ///

    PassiveTorqueLinear(
        const utils::Scalar& T0,
        const utils::Scalar& slope,
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~PassiveTorqueLinear();

    ///
    /// \brief Deep copy of the linear passive torque
    /// \return A deep copy of the linear passive torque
    ///
    PassiveTorqueLinear DeepCopy() const;

    ///
    /// \brief Deep copy of the linear passive torque from another linear passive torque
    /// \param other The linear passive torque to copy
    ///
    void DeepCopy(
        const PassiveTorqueLinear& other);


    ///
    /// \brief Return the maximal torque at a given Q
    /// \param Q The generalized coordinates of the passive torque
    /// \return The maximal torque
    ///
    virtual utils::Scalar passiveTorque();

    ///
    /// \brief Return the maximal torque at a given Q
    /// \param Q The generalized coordinates of the passive torque
    /// \return The maximal torque
    ///
    virtual utils::Scalar passiveTorque(
        const rigidbody::GeneralizedCoordinates &Q) const;

protected:

    ///
    /// \brief Set the type of passive torque
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
