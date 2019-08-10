#ifndef S2M_ACTUATOR_LINEAR_H
#define S2M_ACTUATOR_LINEAR_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

class s2mGenCoord;
class BIORBD_API s2mActuatorLinear : public s2mActuator
{
    public:
        s2mActuatorLinear(int direction,
                          double T0,
                          double pente,
                          unsigned int dofIdx,
                          const s2mString &jointName = "");
        virtual ~s2mActuatorLinear();
        virtual double torqueMax(const s2mGenCoord &Q) const;

    protected:

        virtual void setType(){m_type = "Linear";}             // Quel type d'actuator


        // mx+b
        double m_m;      // Pente
        double m_b; // Torque à zéro


private:
};
#endif // S2M_ACTUATOR_LINEAR_H
