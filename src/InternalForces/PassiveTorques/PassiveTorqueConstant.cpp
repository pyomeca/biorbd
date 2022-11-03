#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorqueConstant.h"

#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant() :
    internal_forces::passive_torques::PassiveTorque(),
    m_Tmax(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    const internal_forces::passive_torques::PassiveTorqueConstant &other) :
    internal_forces::passive_torques::PassiveTorque(other),
    m_Tmax(other.m_Tmax)
{

}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx) :
    internal_forces::passive_torques::PassiveTorque(direction, dofIdx),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueConstant::PassiveTorqueConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx,
    const utils::String &jointName) :
    internal_forces::passive_torques::PassiveTorque(direction, dofIdx, jointName),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
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
    *m_Tmax = *other.m_Tmax;
}

utils::Scalar internal_forces::passive_torques::PassiveTorqueConstant::torqueMax()
{
    return *m_Tmax;
}

void internal_forces::passive_torques::PassiveTorqueConstant::setType()
{
    *m_type = internal_forces::passive_torques::TYPE::CONSTANT;
}
