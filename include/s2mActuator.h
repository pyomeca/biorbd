#ifndef S2M_ACTUATOR_H
#define S2M_ACTUATOR_H

#include "biorbdConfig.h"
    #include "s2mString.h"
    
class BIORBD_API s2mActuator
{
    public:
        s2mActuator(int direction, unsigned int dofIdx, const s2mString &jointName = "");
        virtual ~s2mActuator();
        unsigned int index() const;
        int direction() const;


    protected:
        virtual void setType() = 0;                 // Quel type d'actuator
        s2mString m_type;                           // Type d'actuator
        int m_direction;      // +1 ou -1

        s2mString m_jointName;                     // Nom du parent
        unsigned int m_dofIdx;                        // Index du dof associé



private:
};
#endif // S2M_ACTUATOR_H
