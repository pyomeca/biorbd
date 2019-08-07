#define BIORBD_API_EXPORTS
#include "s2mActuator.h"

#include "s2mError.h"

s2mActuator::s2mActuator(int direction, unsigned int dofIdx, const s2mString &jointName) :
    m_direction(direction),
    m_jointName(jointName),
    m_dofIdx(dofIdx)
{
    s2mError::s2mAssert(m_direction==-1 || m_direction==1, "Direction should be -1 or 1");
}

s2mActuator::~s2mActuator()
{

}

unsigned int s2mActuator::index() const
{
    return m_dofIdx;
}

int s2mActuator::direction() const
{
    return m_direction;
}
