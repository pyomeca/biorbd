#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorConstant.h"

#include "Utils/String.h"
#include "Utils/GenCoord.h"

namespace biorbd { namespace actuator {

ActuatorConstant::ActuatorConstant(
    int direction,
    double Tmax,
    unsigned int dofIdx, const s2mString &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_Tmax(Tmax)
{

}

ActuatorConstant::~ActuatorConstant()
{

}


double ActuatorConstant::torqueMax(){
    return m_Tmax;
}

}}
