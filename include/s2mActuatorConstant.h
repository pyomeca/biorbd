#ifndef S2MACTUATORCONSTANT_H
#define S2MACTUATORCONSTANT_H

#include "biorbdConfig.h"
#include "s2mString.h"
#include "s2mActuator.h"
#include "s2mGenCoord.h"

class BIORBD_API s2mActuatorConstant : public s2mActuator
{
    public:
        s2mActuatorConstant(int direction,
                           double Tmax,
                           unsigned int dofIdx,
                           const s2mString &jointName = "");
        ~s2mActuatorConstant(){}
        virtual double torqueMax();

    protected:

        virtual void setType(){m_type = "Constant";}             // Quel type d'actuator


        double m_Tmax;      // Maximum torque that can be done

private:
};
#endif // S2MACTUATORCONSTANT_H
