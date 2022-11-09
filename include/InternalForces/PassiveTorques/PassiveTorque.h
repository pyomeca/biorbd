#ifndef BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_H
#define BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_H

#include <memory>
#include "biorbdConfig.h"
#include "InternalForces/PassiveTorques/PassiveTorqueEnums.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;
}
namespace internal_forces
{
namespace passive_torques
{

///
/// \brief Class PassiveTorque
///
class BIORBD_API PassiveTorque
{
public:

    ///
    /// \brief Construct passive torque
    ///
    PassiveTorque();

    ///
    /// \brief Construct passive torque from another passive torque
    /// \param other The other passive torque
    ///
    PassiveTorque(
        const PassiveTorque& other);

    ///
    /// \brief Construct passive torque
    /// \param dofIdx Index of the DoF associated with passive torque
    ///
    PassiveTorque(
        unsigned int dofIdx);

    ///
    /// \brief Construct passive torque
    /// \param dofIdx Index of the DoF associated with passive torque
    /// \param jointName The name of the parent joint
    ///
    PassiveTorque(
        unsigned int dofIdx,
        const utils::String &jointName);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~PassiveTorque();

    ///
    /// \brief Deep copy of passive torque
    /// \param other The passive torque to copy
    ///
    void DeepCopy(
        const PassiveTorque& other);

    ///
    /// \brief Return the index of the DoF associated with passive torque
    /// \return The index of the DoF associated with passive torque
    ///
    unsigned int index() const;

    ///
    /// \brief Return the type of the passive torque
    /// \return The type of the passive torque
    ///
    TYPE type() const;

    ///
    /// \brief Return the type of the dof of the passive torque
    /// \return The type of dof of the passive torque
    ///
    DOF_TYPE dof_type() const;

    ///
    /// \brief Return the maximal torque
    /// \return The maximal torque
    ///
    virtual utils::Scalar passiveTorque() = 0;

protected:
    ///
    /// \brief Set the type of passive torque
    ///
    virtual void setType() = 0;
//    virtual void setDofType() = 0;

    std::shared_ptr<TYPE> m_type; ///< The type of the passive torque
    std::shared_ptr<DOF_TYPE> m_dof_type; ///< The type of the dof of the passive torque

    std::shared_ptr<utils::String>
    m_jointName; ///< Name of the parent joint
    std::shared_ptr<unsigned int>
    m_dofIdx;///< Index of the DoF associated with the passive torque

};

}
}
}

#endif // BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_H
