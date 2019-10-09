#ifndef BIORBD_ACTUATORS_ACTUATOR_H
#define BIORBD_ACTUATORS_ACTUATOR_H

#include <memory>
#include "biorbdConfig.h"
#include "Actuators/ActuatorEnums.h"

namespace biorbd {
namespace utils {
class String;
}

namespace actuator {

class BIORBD_API Actuator
{
public:
    Actuator();
    Actuator(const biorbd::actuator::Actuator& other);
    Actuator(
            int direction,
            unsigned int dofIdx);
    Actuator(
            int direction,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName);
    virtual ~Actuator();
    void DeepCopy(const biorbd::actuator::Actuator& other);

    unsigned int index() const;
    int direction() const;

    biorbd::actuator::TYPE type() const;

protected:
    virtual void setType() = 0;                 // Quel type d'actuator

    std::shared_ptr<biorbd::actuator::TYPE> m_type;                           // Type d'actuator
    std::shared_ptr<int> m_direction;      // +1 ou -1

    std::shared_ptr<biorbd::utils::String> m_jointName;                     // Nom du parent
    std::shared_ptr<unsigned int> m_dofIdx;                        // Index du dof associé

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_H
