#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorConstant.h"

#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

actuator::ActuatorConstant::ActuatorConstant() :
    Actuator(),
    m_Tmax(std::make_shared<utils::Scalar>(0))
{
    setType();
}

actuator::ActuatorConstant::ActuatorConstant(
    const actuator::ActuatorConstant &other) :
    Actuator(other),
    m_Tmax(other.m_Tmax)
{

}

actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
{
    setType();
}

actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const utils::Scalar& Tmax,
    unsigned int dofIdx,
    const utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax))
{
    setType();
}

actuator::ActuatorConstant
actuator::ActuatorConstant::DeepCopy() const
{
    actuator::ActuatorConstant copy;
    copy.DeepCopy(*this);
    return copy;
}

void actuator::ActuatorConstant::DeepCopy(const
        actuator::ActuatorConstant &other)
{
    actuator::Actuator::DeepCopy(other);
    *m_Tmax = *other.m_Tmax;
}

utils::Scalar actuator::ActuatorConstant::torqueMax()
{
    return *m_Tmax;
}

void actuator::ActuatorConstant::setType()
{
    *m_type = actuator::TYPE::CONSTANT;
}
