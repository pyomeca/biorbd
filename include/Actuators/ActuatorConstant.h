#ifndef BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
#define BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd { namespace actuator {
class BIORBD_API ActuatorConstant : public Actuator
{
public:
    ActuatorConstant(
            int direction,
            double Tmax,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName = "");
    virtual ~ActuatorConstant();
    virtual double torqueMax();

protected:

    virtual void setType();             // Quel type d'actuator
    double m_Tmax;      // Maximum torque that can be done

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
