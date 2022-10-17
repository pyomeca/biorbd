#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorConstant.h"

#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internalforce::actuator::ActuatorConstant::ActuatorConstant() :
    Actuator(),
    m_Tmax(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internalforce::actuator::ActuatorConstant::ActuatorConstant(
    const internalforce::actuator::ActuatorConstant &other) :
    Actuator(other),
    m_Tmax(other.m_Tmax)
{

}

internalforce::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
{
    setType();
}

internalforce::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx,
    const utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
{
    setType();
}

internalforce::actuator::ActuatorConstant
internalforce::actuator::ActuatorConstant::DeepCopy() const
{
    internalforce::actuator::ActuatorConstant copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::actuator::ActuatorConstant::DeepCopy(const
        internalforce::actuator::ActuatorConstant &other)
{
    internalforce::actuator::Actuator::DeepCopy(other);
    *m_Tmax = *other.m_Tmax;
}

utils::Scalar internalforce::actuator::ActuatorConstant::torqueMax()
{
    return *m_Tmax;
}

void internalforce::actuator::ActuatorConstant::setType()
{
    *m_type = internalforce::actuator::TYPE::CONSTANT;
}
