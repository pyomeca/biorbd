#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorGauss6p.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "RigidBody/GeneralizedCoordinates.h"

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p() :
    biorbd::actuator::Actuator(),
    m_k(std::make_shared<double>(4.3)),
    m_Tmax(std::make_shared<double>(0)),
    m_T0(std::make_shared<double>(0)),
    m_wmax(std::make_shared<double>(0)),
    m_wc(std::make_shared<double>(0)),
    m_amax(std::make_shared<double>(1.0)),
    m_amin(std::make_shared<double>(0)),
    m_wr(std::make_shared<double>(0)),
    m_w1(std::make_shared<double>(0)),
    m_r(std::make_shared<double>(0)),
    m_qopt(std::make_shared<double>(0)),
    m_facteur(std::make_shared<double>(0)),
    m_r2(std::make_shared<double>(0)),
    m_qopt2(std::make_shared<double>(0))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
        const biorbd::actuator::ActuatorGauss6p &other) :
    biorbd::actuator::Actuator(other),
    m_k(other.m_k),
    m_Tmax(other.m_Tmax),
    m_T0(other.m_T0),
    m_wmax(other.m_wmax),
    m_wc(other.m_wc),
    m_amax(other.m_amax),
    m_amin(other.m_amin),
    m_wr(other.m_wr),
    m_w1(other.m_w1),
    m_r(other.m_r),
    m_qopt(other.m_qopt),
    m_facteur(other.m_facteur),
    m_r2(other.m_r2),
    m_qopt2(other.m_qopt2)
{

}
biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
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
        unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_k(std::make_shared<double>(4.3)),
    m_Tmax(std::make_shared<double>(Tmax)),
    m_T0(std::make_shared<double>(T0)),
    m_wmax(std::make_shared<double>(wmax)),
    m_wc(std::make_shared<double>(wc)),
    m_amax(std::make_shared<double>(1.0)),
    m_amin(std::make_shared<double>(amin)),
    m_wr(std::make_shared<double>(wr)),
    m_w1(std::make_shared<double>(w1)),
    m_r(std::make_shared<double>(r)),
    m_qopt(std::make_shared<double>(qopt)),
    m_facteur(std::make_shared<double>(facteur)),
    m_r2(std::make_shared<double>(r2)),
    m_qopt2(std::make_shared<double>(qopt2))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
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
        unsigned int dofIdx,
        const biorbd::utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_k(std::make_shared<double>(4.3)),
    m_Tmax(std::make_shared<double>(Tmax)),
    m_T0(std::make_shared<double>(T0)),
    m_wmax(std::make_shared<double>(wmax)),
    m_wc(std::make_shared<double>(wc)),
    m_amax(std::make_shared<double>(1.0)),
    m_amin(std::make_shared<double>(amin)),
    m_wr(std::make_shared<double>(wr)),
    m_w1(std::make_shared<double>(w1)),
    m_r(std::make_shared<double>(r)),
    m_qopt(std::make_shared<double>(qopt)),
    m_facteur(std::make_shared<double>(facteur)),
    m_r2(std::make_shared<double>(r2)),
    m_qopt2(std::make_shared<double>(qopt2))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::~ActuatorGauss6p()
{

}

biorbd::actuator::ActuatorGauss6p biorbd::actuator::ActuatorGauss6p::DeepCopy() const
{
    biorbd::actuator::ActuatorGauss6p copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorGauss6p::DeepCopy(
        const biorbd::actuator::ActuatorGauss6p &other)
{
    biorbd::actuator::Actuator::DeepCopy(other);
    *m_k = *other.m_k;
    *m_Tmax = *other.m_Tmax;
    *m_T0 = *other.m_T0;
    *m_wmax = *other.m_wmax;
    *m_wc = *other.m_wc;
    *m_amax = *other.m_amax;
    *m_amin = *other.m_amin;
    *m_wr = *other.m_wr;
    *m_w1 = *other.m_w1;
    *m_r = *other.m_r;
    *m_qopt = *other.m_qopt;
    *m_facteur = *other.m_facteur;
    *m_r2 = *other.m_r2;
    *m_qopt2 = *other.m_qopt2;
}


double biorbd::actuator::ActuatorGauss6p::torqueMax(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot){
    double pos(Q[*m_dofIdx] * 180/M_PI);
    double speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Tetanic torque max
    double Tc = *m_T0 * *m_wc / *m_wmax;
    double C = Tc*(*m_wmax + *m_wc); // concentric
    double we = ( (*m_Tmax - *m_T0) * *m_wmax * *m_wc )   /    ( *m_k * *m_T0 * (*m_wmax + *m_wc) );
    double E = -( *m_Tmax - *m_T0 ) * we; // eccentric

    double Tw;
    if (speed >= 0)
        Tw = C / ( *m_wc + speed )  - Tc; // For the concentric
    else
        Tw = E / ( we - speed ) + *m_Tmax; // For the eccentric

    // Differential activation
    double A = *m_amin + ( *m_amax - *m_amin ) / ( 1 + exp( -(speed - *m_w1) / *m_wr   ) );

    // Torque angle
    double Ta =           exp( -(*m_qopt - pos) * (*m_qopt - pos)  /  (2* *m_r * *m_r )   )
            + *m_facteur * exp( -(*m_qopt2 - pos) * (*m_qopt2 - pos)  /  (2 * *m_r2 * *m_r2)   );

    // Calculation of the max torque
    return Tw * A * Ta;
}

void biorbd::actuator::ActuatorGauss6p::setType()
{
    *m_type = biorbd::actuator::TYPE::GAUSS6P;
}
