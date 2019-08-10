#define BIORBD_API_EXPORTS
#include "Actuators/Actuator.h"

#include "Utils/Error.h"

namespace biorbd { namespace actuator {

Actuator::Actuator(int direction, unsigned int dofIdx, const s2mString &jointName) :
    m_direction(direction),
    m_jointName(jointName),
    m_dofIdx(dofIdx)
{
    s2mError::s2mAssert(m_direction==-1 || m_direction==1, "Direction should be -1 or 1");
}

Actuator::~Actuator()
{

}

unsigned int Actuator::index() const
{
    return m_dofIdx;
}

int Actuator::direction() const
{
    return m_direction;
}

}}
