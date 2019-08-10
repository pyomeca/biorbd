#ifndef S2M_ACTUATOR_CONSTANT_H
#define S2M_ACTUATOR_CONSTANT_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

class BIORBD_API s2mActuatorConstant : public s2mActuator
{
    public:
        s2mActuatorConstant(int direction,
                           double Tmax,
                           unsigned int dofIdx,
                           const s2mString &jointName = "");
        virtual ~s2mActuatorConstant();
        virtual double torqueMax();

    protected:

        virtual void setType(){m_type = "Constant";}             // Quel type d'actuator


        double m_Tmax;      // Maximum torque that can be done

};
#endif // S2M_ACTUATOR_CONSTANT_H
