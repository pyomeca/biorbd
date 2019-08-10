#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorGauss3p.h"

#include "s2mGenCoord.h"

namespace biorbd { namespace actuator {

s2mActuatorGauss3p::s2mActuatorGauss3p(
    int direction,
    double Tmax,
    double T0,
    double wmax,
    double wc,
    double amin,
    double wr,
    double w1,
    double r,
    double qopt,
    unsigned int dofIdx, const s2mString &jointName) :
    s2mActuator(direction, dofIdx, jointName),
    m_k(4.3), // Valeur par défaut
    m_Tmax(Tmax),
    m_T0(T0),
    m_wmax(wmax),
    m_wc(wc),
    m_amax(1.0), // Valeur par défaut
    m_amin(amin),
    m_wr(wr),
    m_w1(w1),
    m_r(r),
    m_qopt(qopt)
{

}

s2mActuatorGauss3p::~s2mActuatorGauss3p()
{

}


double s2mActuatorGauss3p::torqueMax(const s2mGenCoord &Q, const s2mGenCoord &Qdot){
    double pos(Q[m_dofIdx] * 180/M_PI);
    double speed(Qdot[m_dofIdx] * 180/M_PI);

    // Tetanic torque max
    double Tc = m_T0*m_wc/m_wmax;
    double C = Tc*(m_wmax+m_wc); // en concentrique
    double we = ( (m_Tmax-m_T0) * m_wmax * m_wc )   /    ( m_k * m_T0 * (m_wmax+m_wc) );
    double E = -( m_Tmax - m_T0 ) * we; // en excentrique

    double Tw; // Initiation d'une variable
    if (speed >= 0)
        Tw = C / ( m_wc + speed )  - Tc; // Pour le concentrique
    else
        Tw = E / ( we - speed ) + m_Tmax; // Pour l'excentrique


    // Differential activation
    double A = m_amin + ( m_amax - m_amin ) / ( 1 + exp( -(speed-m_w1) / m_wr   ) );

    // Torque angle
    double Ta = exp( -(m_qopt-pos) * (m_qopt-pos)   /   (2*m_r*m_r)   );

    // Calcul du couple max
    return Tw * A * Ta;

}

}}
