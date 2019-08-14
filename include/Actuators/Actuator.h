#ifndef BIORBD_ACTUATORS_ACTUATOR_H
#define BIORBD_ACTUATORS_ACTUATOR_H

#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd { namespace actuator {
class BIORBD_API Actuator
{
public:
    Actuator(
            int direction,
            unsigned int dofIdx,
            const biorbd::utils::String &jointName = "");
    virtual ~Actuator();
    unsigned int index() const;
    int direction() const;


protected:
    virtual void setType() = 0;                 // Quel type d'actuator
    biorbd::utils::String m_type;                           // Type d'actuator
    int m_direction;      // +1 ou -1

    biorbd::utils::String m_jointName;                     // Nom du parent
    unsigned int m_dofIdx;                        // Index du dof associé

};

}}

#endif // BIORBD_ACTUATORS_ACTUATOR_H
