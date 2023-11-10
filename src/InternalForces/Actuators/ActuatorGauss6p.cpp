#define BIORBD_API_EXPORTS
#include "InternalForces/Actuators/ActuatorGauss6p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internal_forces::actuator::ActuatorGauss6p::ActuatorGauss6p() :
    internal_forces::actuator::Actuator(),
    m_k(std::make_shared<utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<utils::Scalar>(0)),
    m_T0(std::make_shared<utils::Scalar>(0)),
    m_wmax(std::make_shared<utils::Scalar>(0)),
    m_wc(std::make_shared<utils::Scalar>(0)),
    m_amax(std::make_shared<utils::Scalar>(1.0)),
    m_amin(std::make_shared<utils::Scalar>(0)),
    m_wr(std::make_shared<utils::Scalar>(0)),
    m_w1(std::make_shared<utils::Scalar>(0)),
    m_r(std::make_shared<utils::Scalar>(0)),
    m_qopt(std::make_shared<utils::Scalar>(0)),
    m_facteur(std::make_shared<utils::Scalar>(0)),
    m_r2(std::make_shared<utils::Scalar>(0)),
    m_qopt2(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::actuator::ActuatorGauss6p::ActuatorGauss6p(
    const internal_forces::actuator::ActuatorGauss6p &other) :
    internal_forces::actuator::Actuator(other),
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
internal_forces::actuator::ActuatorGauss6p::ActuatorGauss6p(
    int direction,
    const utils::Scalar& Tmax,
    const utils::Scalar& T0,
    const utils::Scalar& wmax,
    const utils::Scalar& wc,
    const utils::Scalar& amin,
    const utils::Scalar& wr,
    const utils::Scalar& w1,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    const utils::Scalar& facteur,
    const utils::Scalar& r2,
    const utils::Scalar& qopt2,
    size_t dofIdx) :
    Actuator(direction, dofIdx),
    m_k(std::make_shared<utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax)),
    m_T0(std::make_shared<utils::Scalar>(T0)),
    m_wmax(std::make_shared<utils::Scalar>(wmax)),
    m_wc(std::make_shared<utils::Scalar>(wc)),
    m_amax(std::make_shared<utils::Scalar>(1.0)),
    m_amin(std::make_shared<utils::Scalar>(amin)),
    m_wr(std::make_shared<utils::Scalar>(wr)),
    m_w1(std::make_shared<utils::Scalar>(w1)),
    m_r(std::make_shared<utils::Scalar>(r)),
    m_qopt(std::make_shared<utils::Scalar>(qopt)),
    m_facteur(std::make_shared<utils::Scalar>(facteur)),
    m_r2(std::make_shared<utils::Scalar>(r2)),
    m_qopt2(std::make_shared<utils::Scalar>(qopt2))
{
    setType();
}

internal_forces::actuator::ActuatorGauss6p::ActuatorGauss6p(
    int direction,
    const utils::Scalar& Tmax,
    const utils::Scalar& T0,
    const utils::Scalar& wmax,
    const utils::Scalar& wc,
    const utils::Scalar& amin,
    const utils::Scalar& wr,
    const utils::Scalar& w1,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    const utils::Scalar& facteur,
    const utils::Scalar& r2,
    const utils::Scalar& qopt2,
    size_t dofIdx,
    const utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_k(std::make_shared<utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<utils::Scalar>(Tmax)),
    m_T0(std::make_shared<utils::Scalar>(T0)),
    m_wmax(std::make_shared<utils::Scalar>(wmax)),
    m_wc(std::make_shared<utils::Scalar>(wc)),
    m_amax(std::make_shared<utils::Scalar>(1.0)),
    m_amin(std::make_shared<utils::Scalar>(amin)),
    m_wr(std::make_shared<utils::Scalar>(wr)),
    m_w1(std::make_shared<utils::Scalar>(w1)),
    m_r(std::make_shared<utils::Scalar>(r)),
    m_qopt(std::make_shared<utils::Scalar>(qopt)),
    m_facteur(std::make_shared<utils::Scalar>(facteur)),
    m_r2(std::make_shared<utils::Scalar>(r2)),
    m_qopt2(std::make_shared<utils::Scalar>(qopt2))
{
    setType();
}

internal_forces::actuator::ActuatorGauss6p::~ActuatorGauss6p()
{

}

internal_forces::actuator::ActuatorGauss6p internal_forces::actuator::ActuatorGauss6p::DeepCopy()
const
{
    internal_forces::actuator::ActuatorGauss6p copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::actuator::ActuatorGauss6p::DeepCopy(
    const internal_forces::actuator::ActuatorGauss6p &other)
{
    internal_forces::actuator::Actuator::DeepCopy(other);
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

utils::Scalar internal_forces::actuator::ActuatorGauss6p::torqueMax()
{
    utils::Error::raise("torqueMax for ActuatorGauss6p must be called with Q and Qdot");
}

utils::Scalar internal_forces::actuator::ActuatorGauss6p::torqueMax(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    utils::Scalar pos(Q[static_cast<unsigned int>(*m_dofIdx)] * 180/M_PI);
    utils::Scalar speed(Qdot[static_cast<unsigned int>(*m_dofIdx)] * 180/M_PI);

    // Tetanic torque max
    utils::Scalar Tc = *m_T0 * *m_wc / *m_wmax;
    utils::Scalar C = Tc*(*m_wmax + *m_wc); // concentric
    utils::Scalar we = ( (*m_Tmax - *m_T0) * *m_wmax * *m_wc )   /
                               ( *m_k * *m_T0 * (*m_wmax + *m_wc) );
    utils::Scalar E = -( *m_Tmax - *m_T0 ) * we; // eccentric

    utils::Scalar Tw;
#ifdef BIORBD_USE_CASADI_MATH
    Tw = IF_ELSE_NAMESPACE::if_else(IF_ELSE_NAMESPACE::ge(speed, 0),
                             C / ( *m_wc + speed )  - Tc,
                             E / ( we - speed ) + *m_Tmax);
#else
    if (speed >= 0) {
        Tw = C / ( *m_wc + speed )  - Tc;    // For the concentric
    } else {
        Tw = E / ( we - speed ) + *m_Tmax;    // For the eccentric
    }
#endif

    // Differential activation
    utils::Scalar A =
        *m_amin + ( *m_amax - *m_amin )
        / ( 1 + utils::Scalar(exp( -(speed - *m_w1) / *m_wr   )) );

    // Torque angle
    utils::Scalar Ta = utils::Scalar(exp( -(*m_qopt - pos) *
                               (*m_qopt - pos)  /  (2* *m_r * *m_r )   ))
                               + *m_facteur * utils::Scalar(exp( -(*m_qopt2 - pos) *
                                       (*m_qopt2 - pos)  /  (2 * *m_r2 * *m_r2)   ));

    // Calculation of the max torque
    return Tw * A * Ta;
}

void internal_forces::actuator::ActuatorGauss6p::setType()
{
    *m_type = internal_forces::actuator::TYPE::GAUSS6P;
}
