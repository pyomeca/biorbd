#define BIORBD_API_EXPORTS
#include "s2mActuatorLinear.h"

#include "s2mGenCoord.h"

s2mActuatorLinear::s2mActuatorLinear(
    int direction,
    double T0,
    double pente,
    unsigned int dofIdx, const s2mString &jointName) :
    s2mActuator(direction, dofIdx, jointName),
    m_m(pente),
    m_b(T0)
{

}


double s2mActuatorLinear::torqueMax(const s2mGenCoord &Q) const {
//    std::cout << "Q[" << m_dofIdx << "] = " << Q[m_dofIdx] << std::endl;
//    std::cout << "M = " << m_m << std::endl;
//    std::cout << "B = " << m_b << std::endl;
//    std::cout << "Torque = " << (Q[m_dofIdx]*180/PI)*m_m+m_b << std::endl << std::endl;

    return (Q[m_dofIdx]*180/M_PI)*m_m+m_b;
}
