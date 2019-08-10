#ifndef S2M_ACTUATOR_CONSTANT_H
#define S2M_ACTUATOR_CONSTANT_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd { namespace actuator {
class BIORBD_API ActuatorConstant : public Actuator
{
    public:
        ActuatorConstant(int direction,
                           double Tmax,
                           unsigned int dofIdx,
                           const s2mString &jointName = "");
        virtual ~ActuatorConstant();
        virtual double torqueMax();

    protected:

        virtual void setType(){m_type = "Constant";}             // Quel type d'actuator


        double m_Tmax;      // Maximum torque that can be done

};

}}

#endif // S2M_ACTUATOR_CONSTANT_H
