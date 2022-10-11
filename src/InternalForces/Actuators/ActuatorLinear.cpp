#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorLinear.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;
using namespace internalforce;

actuator::ActuatorLinear::ActuatorLinear() :
    Actuator(),
    m_m(std::make_shared<utils::Scalar>(0)),
    m_b(std::make_shared<utils::Scalar>(0))
{
    setType();
}

actuator::ActuatorLinear::ActuatorLinear(
    const actuator::ActuatorLinear &other) :
    Actuator(other),
    m_m(other.m_m),
    m_b(other.m_b)
{

}

actuator::ActuatorLinear::ActuatorLinear(
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

actuator::ActuatorLinear::ActuatorLinear(
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

actuator::ActuatorLinear::~ActuatorLinear()
{

}

actuator::ActuatorLinear actuator::ActuatorLinear::DeepCopy()
const
{
    actuator::ActuatorLinear copy;
    copy.DeepCopy(*this);
    return copy;
}

void actuator::ActuatorLinear::DeepCopy(const
        actuator::ActuatorLinear &other)
{
    actuator::Actuator::DeepCopy(other);
    *m_m = *other.m_m;
    *m_b = *other.m_b;
}

utils::Scalar actuator::ActuatorLinear::torqueMax()
{
    utils::Error::raise("torqueMax for ActuatorLinear must be called with Q and Qdot");
}


utils::Scalar actuator::ActuatorLinear::torqueMax(
    const rigidbody::GeneralizedCoordinates &Q) const
{
    return (Q[*m_dofIdx]*180/M_PI) * *m_m + *m_b;
}

void actuator::ActuatorLinear::setType()
{
    *m_type = actuator::TYPE::LINEAR;
}
