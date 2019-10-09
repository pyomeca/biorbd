#ifndef BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
#define BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd {
namespace actuator {

class BIORBD_API ActuatorConstant : public Actuator
{
public:
    ActuatorConstant();
    ActuatorConstant(const biorbd::actuator::ActuatorConstant& other);
    ActuatorConstant(
            int direction,
            double Tmax,
            unsigned int dofIdx);
    ActuatorConstant(
            int direction,
            double Tmax,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);
    biorbd::actuator::ActuatorConstant DeepCopy() const;
    void DeepCopy(const biorbd::actuator::ActuatorConstant& other);

    virtual double torqueMax();

protected:

    virtual void setType();             // Quel type d'actuator
    std::shared_ptr<double> m_Tmax;      // Maximum torque that can be done

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
