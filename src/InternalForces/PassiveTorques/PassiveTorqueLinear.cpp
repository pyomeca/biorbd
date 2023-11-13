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
    setType();
}

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear(
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    size_t dofIdx) :
    internal_forces::passive_torques::PassiveTorque(dofIdx),
    m_m(std::make_shared<utils::Scalar>(slope)),
    m_b(std::make_shared<utils::Scalar>(T0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueLinear::PassiveTorqueLinear(
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    size_t dofIdx,
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

utils::Scalar internal_forces::passive_torques::PassiveTorqueLinear::passiveTorque()
{
    utils::Error::raise("passiveTorque for PassiveTorqueLinear must be called with Q");
}


utils::Scalar internal_forces::passive_torques::PassiveTorqueLinear::passiveTorque(
    const rigidbody::GeneralizedCoordinates &Q) const
{
    return Q[static_cast<unsigned int>(*m_dofIdx)] * *m_m + *m_b;
}

void internal_forces::passive_torques::PassiveTorqueLinear::setType()
{
    *m_type = internal_forces::passive_torques::TORQUE_TYPE::TORQUE_LINEAR;
}
