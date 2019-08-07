#define BIORBD_API_EXPORTS
#include "s2mActuatorGauss6p.h"

#include "s2mGenCoord.h"

s2mActuatorGauss6p::s2mActuatorGauss6p(
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
    double facteur,
    double r2,
    double qopt2,
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
    m_qopt(qopt),
    m_facteur(facteur),
    m_r2(r2),
    m_qopt2(qopt2)
{

}


double s2mActuatorGauss6p::torqueMax(const s2mGenCoord &Q, const s2mGenCoord &Qdot){
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
    double Ta =           exp( -(m_qopt -pos) * (m_qopt -pos)  /  (2*m_r *m_r )   )
            + m_facteur * exp( -(m_qopt2-pos) * (m_qopt2-pos)  /  (2*m_r2*m_r2)   );

    // Calcul du couple max
    return Tw * A * Ta;


}
