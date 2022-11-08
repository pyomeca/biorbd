#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorqueLinear.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear() :
    internal_forces::passive_torques::PassiveTorque(),
    m_m(std::make_shared<utils::Scalar>(0)),
    m_b(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear(
    const internal_forces::passive_torques::PassiveTorqueLinear &other) :
    internal_forces::passive_torques::PassiveTorque(other),
    m_m(other.m_m),
    m_b(other.m_b)
{

}

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear(
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    unsigned int dofIdx) :
    internal_forces::passive_torques::PassiveTorque(dofIdx),
    m_m(std::make_shared<utils::Scalar>(slope)),
    m_b(std::make_shared<utils::Scalar>(T0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear(
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    unsigned int dofIdx,
    const utils::String &jointName) :
    internal_forces::passive_torques::PassiveTorque(dofIdx, jointName),
    m_m(std::make_shared<utils::Scalar>(slope)),
    m_b(std::make_shared<utils::Scalar>(T0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueLinear::~PassiveTorqueLinear()
{

}

internal_forces::passive_torques::PassiveTorqueLinear internal_forces::passive_torques::PassiveTorqueLinear::DeepCopy()
const
{
    internal_forces::passive_torques::PassiveTorqueLinear copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::passive_torques::PassiveTorqueLinear::DeepCopy(const
        internal_forces::passive_torques::PassiveTorqueLinear &other)
{
    internal_forces::passive_torques::PassiveTorque::DeepCopy(other);
    *m_m = *other.m_m;
    *m_b = *other.m_b;
}

const utils::Scalar& internal_forces::passive_torques::PassiveTorqueLinear::passiveTorque()
{
    utils::Error::raise("passiveTorque for PassiveTorqueLinear must be called with Q");
}


const utils::Scalar& internal_forces::passive_torques::PassiveTorqueLinear::passiveTorque(
    const rigidbody::GeneralizedCoordinates &Q)
{
    return (Q[*m_dofIdx]*180/M_PI);
}

void internal_forces::passive_torques::PassiveTorqueLinear::setType()
{
    *m_type = internal_forces::passive_torques::TYPE::LINEAR;
}
