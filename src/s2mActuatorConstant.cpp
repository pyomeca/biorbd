#define BIORBD_API_EXPORTS
#include "s2mActuatorConstant.h"

#include "s2mString.h"
#include "s2mGenCoord.h"

s2mActuatorConstant::s2mActuatorConstant(
    int direction,
    double Tmax,
    unsigned int dofIdx, const s2mString &jointName) :
    s2mActuator(direction, dofIdx, jointName),
    m_Tmax(Tmax)
{

}


double s2mActuatorConstant::torqueMax(){
    return m_Tmax;
}
