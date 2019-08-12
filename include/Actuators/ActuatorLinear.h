#ifndef S2M_ACTUATOR_LINEAR_H
#define S2M_ACTUATOR_LINEAR_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd { namespace utils {
class GenCoord;
}}
namespace biorbd { namespace actuator {
class BIORBD_API ActuatorLinear : public Actuator
{
    public:
        ActuatorLinear(int direction,
                          double T0,
                          double pente,
                          unsigned int dofIdx,
                          const s2mString &jointName = "");
        virtual ~ActuatorLinear();
        virtual double torqueMax(const biorbd::utils::GenCoord &Q) const;

    protected:

        virtual void setType(){m_type = "Linear";}             // Quel type d'actuator


        // mx+b
        double m_m;      // Pente
        double m_b; // Torque à zéro

};

}}

#endif // S2M_ACTUATOR_LINEAR_H
