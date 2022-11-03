#ifndef BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_CONSTANT_H
#define BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_CONSTANT_H

#include "biorbdConfig.h"
#include "InternalForces/PassiveTorques/PassiveTorque.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace passive_torques
{

///
/// \brief Class PassiveTorqueConstant is a joint actuator type which maximum is contant
///
class BIORBD_API PassiveTorqueConstant : public PassiveTorque
{
public:
    ///
    /// \brief Construct a constant actuator
    ///
    PassiveTorqueConstant();

    ///
    /// \brief Construct a constant actuator from another actuator
    /// \param other The other constant actuator
    ///
    PassiveTorqueConstant(
        const PassiveTorqueConstant& other);

    ///
    /// \brief Construct a constant actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    PassiveTorqueConstant(
        int direction,
        const utils::Scalar& Tmax,
        unsigned int dofIdx);

    ///
    /// \brief Construct a constant actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName The name of the parent joint
    ///
    PassiveTorqueConstant(
        int direction,
        const utils::Scalar& Tmax,
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Deep copy of the constant actuator
    /// \return A copy of the constant actuator
    ///
    PassiveTorqueConstant DeepCopy() const;

    ///
    /// \brief Deep copy of the constant actuator to another constant actuator
    /// \param other The constant actuator to copy
    ///
    void DeepCopy(
        const PassiveTorqueConstant& other);

    ///
    /// \brief Return the maximal torque
    /// \return The maximal torque
    ///
    virtual utils::Scalar torqueMax();

protected:
    ///
    /// \brief Set the type of the constant actuator
    ///
    virtual void setType();
//    virtual void setDofType();
    std::shared_ptr<utils::Scalar>
    m_Tmax; ///< Maximal torque that can be done

};

}
}
}

#endif // BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_CONSTANT_H
