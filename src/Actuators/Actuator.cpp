#define BIORBD_API_EXPORTS
#include "Actuators/Actuator.h"

#include "Utils/Error.h"

biorbd::actuator::Actuator::Actuator(
        int direction,
        unsigned int dofIdx,
        const biorbd::utils::String &jointName) :
    m_direction(direction),
    m_jointName(jointName),
    m_dofIdx(dofIdx)
{
    biorbd::utils::Error::error(m_direction==-1 || m_direction==1, "Direction should be -1 or 1");
}

biorbd::actuator::Actuator::~Actuator()
{

}

unsigned int biorbd::actuator::Actuator::index() const
{
    return m_dofIdx;
}

int biorbd::actuator::Actuator::direction() const
{
    return m_direction;
}
