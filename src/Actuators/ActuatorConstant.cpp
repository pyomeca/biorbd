#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorConstant.h"

#include "Utils/String.h"
#include "Utils/GenCoord.h"

biorbd::actuator::ActuatorConstant::ActuatorConstant(
        int direction,
        double Tmax,
        unsigned int dofIdx,
        const biorbd::utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_Tmax(Tmax)
{

}

biorbd::actuator::ActuatorConstant::~ActuatorConstant()
{

}


double biorbd::actuator::ActuatorConstant::torqueMax(){
    return m_Tmax;
}

void biorbd::actuator::ActuatorConstant::setType(){m_type = "Constant";}
