#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorGauss3p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p() :
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
    m_qopt(std::make_shared<double>(0))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
        const biorbd::actuator::ActuatorGauss3p &other) :
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
    m_qopt(other.m_qopt)
{

}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
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
        unsigned int dofIdx) :
    biorbd::actuator::Actuator(direction, dofIdx),
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
    m_qopt(std::make_shared<double>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
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
        unsigned int dofIdx,
        const biorbd::utils::String &jointName) :
    biorbd::actuator::Actuator(direction, dofIdx, jointName),
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
    m_qopt(std::make_shared<double>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::~ActuatorGauss3p()
{

}

biorbd::actuator::ActuatorGauss3p biorbd::actuator::ActuatorGauss3p::DeepCopy() const
{
    biorbd::actuator::ActuatorGauss3p copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorGauss3p::DeepCopy(
        const biorbd::actuator::ActuatorGauss3p &other)
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
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss3p::torqueMax()
{
    biorbd::utils::Error::raise("torqueMax for ActuatorGauss3p must be called with Q and Qdot");
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss3p::torqueMax(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot){
    double pos(Q[*m_dofIdx] * 180/M_PI);
    double speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Tetanic torque max
    double Tc = *m_T0 * *m_wc / *m_wmax;
    double C = Tc * (*m_wmax + *m_wc); // concentric
    double we = ( (*m_Tmax - *m_T0) * *m_wmax * *m_wc )   /    ( *m_k * *m_T0 * (*m_wmax + *m_wc) );
    double E = -( *m_Tmax - *m_T0 ) * we; // excentric

    double Tw;
    if (speed >= 0)
        Tw = C / ( *m_wc + speed )  - Tc; // For the concentric
    else
        Tw = E / ( we - speed ) + *m_Tmax; // For the excentric


    // Differential activation
    double A = *m_amin + ( *m_amax - *m_amin ) / ( 1 + exp( -(speed - *m_w1) / *m_wr   ) );

    // Torque angle
    double Ta = exp( -(*m_qopt - pos) * (*m_qopt - pos)   /   (2 * *m_r * *m_r)   );

    // Calculation of the max torque
    return Tw * A * Ta;

}

void biorbd::actuator::ActuatorGauss3p::setType(){
    *m_type = biorbd::actuator::TYPE::GAUSS3P;
}
