#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/Actuator.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internalforce::actuator::Actuator::Actuator() :
    m_type(std::make_shared<internalforce::actuator::TYPE>
           (internalforce::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(0)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(-1))
{

}

internalforce::actuator::Actuator::Actuator(
    const internalforce::actuator::Actuator &other) :
    m_type(other.m_type),
    m_direction(other.m_direction),
    m_jointName(other.m_jointName),
    m_dofIdx(other.m_dofIdx)
{

}

internalforce::actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx) :
    m_type(std::make_shared<internalforce::actuator::TYPE>
           (internalforce::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{

}

internalforce::actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx,
    const utils::String &jointName) :
    m_type(std::make_shared<internalforce::actuator::TYPE>
           (internalforce::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>(jointName)),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{
    utils::Error::check(*m_direction==-1 || *m_direction==1,
                                "Direction should be -1 or 1");
}

internalforce::actuator::Actuator::~Actuator()
{

}

void internalforce::actuator::Actuator::DeepCopy(const internalforce::actuator::Actuator
        &other)
{
    *m_type = *other.m_type;
    *m_direction = *other.m_direction;
    *m_jointName = *other.m_jointName;
    *m_dofIdx = *other.m_dofIdx;
}

unsigned int internalforce::actuator::Actuator::index() const
{
    return *m_dofIdx;
}

int internalforce::actuator::Actuator::direction() const
{
    return *m_direction;
}

internalforce::actuator::TYPE internalforce::actuator::Actuator::type() const
{
    return *m_type;
}
