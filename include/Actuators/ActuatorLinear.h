#ifndef BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
#define BIORBD_ACTUATORS_ACTUATOR_LINEAR_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace actuator {

class BIORBD_API ActuatorLinear : public Actuator
{
public:
    ActuatorLinear();
    ActuatorLinear(
            const biorbd::actuator::ActuatorLinear& other);
    ActuatorLinear(
            int direction,
            double T0,
            double pente,
            unsigned int dofIdx);
    ActuatorLinear(
            int direction,
            double T0,
            double pente,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);
    virtual ~ActuatorLinear();
    biorbd::actuator::ActuatorLinear DeepCopy() const;
    void DeepCopy(const biorbd::actuator::ActuatorLinear& other);

    virtual double torqueMax(const biorbd::rigidbody::GeneralizedCoordinates &Q) const;

protected:

    virtual void setType();             // Quel type d'actuator

    // mx+b
    std::shared_ptr<double> m_m;      // Pente
    std::shared_ptr<double> m_b; // Torque à zéro

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_LINEAR_H
