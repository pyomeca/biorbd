#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorLinear.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internalforce::actuator::ActuatorLinear::ActuatorLinear() :
    Actuator(),
    m_m(std::make_shared<utils::Scalar>(0)),
    m_b(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internalforce::actuator::ActuatorLinear::ActuatorLinear(
    const internalforce::actuator::ActuatorLinear &other) :
    Actuator(other),
    m_m(other.m_m),
    m_b(other.m_b)
{

}

internalforce::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_m(std::make_shared<utils::Scalar>(slope)),
    m_b(std::make_shared<utils::Scalar>(T0))
{
    setType();
}

internalforce::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const utils::Scalar& T0,
    const utils::Scalar& slope,
    unsigned int dofIdx,
    const utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_m(std::make_shared<utils::Scalar>(slope)),
    m_b(std::make_shared<utils::Scalar>(T0))
{
    setType();
}

internalforce::actuator::ActuatorLinear::~ActuatorLinear()
{

}

internalforce::actuator::ActuatorLinear internalforce::actuator::ActuatorLinear::DeepCopy()
const
{
    internalforce::actuator::ActuatorLinear copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::actuator::ActuatorLinear::DeepCopy(const
        internalforce::actuator::ActuatorLinear &other)
{
    internalforce::actuator::Actuator::DeepCopy(other);
    *m_m = *other.m_m;
    *m_b = *other.m_b;
}

utils::Scalar internalforce::actuator::ActuatorLinear::torqueMax()
{
    utils::Error::raise("torqueMax for ActuatorLinear must be called with Q and Qdot");
}


utils::Scalar internalforce::actuator::ActuatorLinear::torqueMax(
    const rigidbody::GeneralizedCoordinates &Q) const
{
    return (Q[*m_dofIdx]*180/M_PI) * *m_m + *m_b;
}

void internalforce::actuator::ActuatorLinear::setType()
{
    *m_type = internalforce::actuator::TYPE::LINEAR;
}
