#define BIORBD_API_EXPORTS
#include "Actuators/Actuator.h"

#include "Utils/Error.h"
#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

actuator::Actuator::Actuator() :
    m_type(std::make_shared<actuator::TYPE>
           (actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(0)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(-1))
{

}

actuator::Actuator::Actuator(
    const actuator::Actuator &other) :
    m_type(other.m_type),
    m_direction(other.m_direction),
    m_jointName(other.m_jointName),
    m_dofIdx(other.m_dofIdx)
{

}

actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx) :
    m_type(std::make_shared<actuator::TYPE>
           (actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{

}

actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx,
    const utils::String &jointName) :
    m_type(std::make_shared<actuator::TYPE>
           (actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<utils::String>(jointName)),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{
    utils::Error::check(*m_direction==-1 || *m_direction==1,
                                "Direction should be -1 or 1");
}

actuator::Actuator::~Actuator()
{

}

void actuator::Actuator::DeepCopy(const actuator::Actuator
        &other)
{
    *m_type = *other.m_type;
    *m_direction = *other.m_direction;
    *m_jointName = *other.m_jointName;
    *m_dofIdx = *other.m_dofIdx;
}

unsigned int actuator::Actuator::index() const
{
    return *m_dofIdx;
}

int actuator::Actuator::direction() const
{
    return *m_direction;
}

actuator::TYPE actuator::Actuator::type() const
{
    return *m_type;
}
