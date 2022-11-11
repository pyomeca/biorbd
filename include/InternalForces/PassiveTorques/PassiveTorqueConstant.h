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
/// \brief Class PassiveTorqueConstant is a joint passive torque type which maximum is contant
///
class BIORBD_API PassiveTorqueConstant : public PassiveTorque
{
public:
    ///
    /// \brief Construct a constant passive torque
    ///
    PassiveTorqueConstant();

    ///
    /// \brief Construct an Constant passive torque from another
    /// \param other The other passive torque
    ///
    PassiveTorqueConstant(
        const PassiveTorqueConstant& other);

    ///
    /// \brief Construct a constant passive torque
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with passive torque
    ///
    PassiveTorqueConstant(
        const utils::Scalar& Torque,
        unsigned int dofIdx);

    ///
    /// \brief Construct a constant passive torque
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with passive torque
    /// \param jointName The name of the parent joint
    ///
    PassiveTorqueConstant(
        const utils::Scalar& Torque,
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Deep copy of the constant passive torque
    /// \return A copy of the constant passive torque
    ///
    PassiveTorqueConstant DeepCopy() const;

    ///
    /// \brief Deep copy of the constant passive torque to another constant passive torque
    /// \param other The constant passive torque to copy
    ///
    void DeepCopy(
        const PassiveTorqueConstant& other);

    ///
    /// \brief Return the maximal torque
    /// \return The maximal torque
    ///
    virtual utils::Scalar passiveTorque();

protected:
    ///
    /// \brief Set the type of the constant passive torque
    ///
    virtual void setType();
//    virtual void setDofType();
    std::shared_ptr<utils::Scalar> m_Torque; ///< Maximal torque that can be done

};

}
}
}

#endif // BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_CONSTANT_H
