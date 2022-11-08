#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorqueConstant.h"

#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant() :
    internal_forces::passive_torques::PassiveTorque(),
    m_Torque(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    const internal_forces::passive_torques::PassiveTorqueConstant &other) :
    internal_forces::passive_torques::PassiveTorque(other),
    m_Torque(other.m_Torque)
{

}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    const utils::Scalar& Torque,
    unsigned int dofIdx) :
    internal_forces::passive_torques::PassiveTorque(dofIdx),
    m_Torque(std::make_shared<utils::Scalar>(Torque))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    const utils::Scalar& Torque,
    unsigned int dofIdx,
    const utils::String &jointName) :
    internal_forces::passive_torques::PassiveTorque(dofIdx, jointName),
    m_Torque(std::make_shared<utils::Scalar>(Torque))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueConstant
internal_forces::passive_torques::PassiveTorqueConstant::DeepCopy() const
{
    internal_forces::passive_torques::PassiveTorqueConstant copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::passive_torques::PassiveTorqueConstant::DeepCopy(const
        internal_forces::passive_torques::PassiveTorqueConstant &other)
{
    internal_forces::passive_torques::PassiveTorque::DeepCopy(other);
    *m_Torque = *other.m_Torque;
}

const utils::Scalar& internal_forces::passive_torques::PassiveTorqueConstant::passiveTorque()
{
    return *m_Torque;
}

void internal_forces::passive_torques::PassiveTorqueConstant::setType()
{
    *m_type = internal_forces::passive_torques::TYPE::CONSTANT;
}
