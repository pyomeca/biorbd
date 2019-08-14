#ifndef BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
#define BIORBD_ACTUATORS_ACTUATOR_LINEAR_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace utils {
class GenCoord;
}

namespace actuator {

class BIORBD_API ActuatorLinear : public Actuator
{
public:
    ActuatorLinear(
            int direction,
            double T0,
            double pente,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName = "");
    virtual ~ActuatorLinear();
    virtual double torqueMax(const biorbd::utils::GenCoord &Q) const;

protected:

    virtual void setType();             // Quel type d'actuator


    // mx+b
    double m_m;      // Pente
    double m_b; // Torque à zéro

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
