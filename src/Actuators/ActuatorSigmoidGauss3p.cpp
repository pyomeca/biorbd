#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorSigmoidGauss3p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p() :
    actuator::Actuator(),
    m_theta(std::make_shared<utils::Scalar>(0)),
    m_lambda(std::make_shared<utils::Scalar>(0)),
    m_offset(std::make_shared<utils::Scalar>(0)),
    m_r(std::make_shared<utils::Scalar>(0)),
    m_qopt(std::make_shared<utils::Scalar>(0))
{
    setType();
}

actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    const actuator::ActuatorSigmoidGauss3p &other) :
    actuator::Actuator(other),
    m_theta(other.m_theta),
    m_lambda(other.m_lambda),
    m_offset(other.m_offset),
    m_r(other.m_r),
    m_qopt(other.m_qopt)
{

}

actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const utils::Scalar& theta,
    const utils::Scalar& lambda,
    const utils::Scalar& offset,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    unsigned int dofIdx) :
    actuator::Actuator(direction, dofIdx),
    m_theta(std::make_shared<utils::Scalar>(theta)),
    m_lambda(std::make_shared<utils::Scalar>(lambda)),
    m_offset(std::make_shared<utils::Scalar>(offset)),
    m_r(std::make_shared<utils::Scalar>(r)),
    m_qopt(std::make_shared<utils::Scalar>(qopt))
{
    setType();
}

actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const utils::Scalar& theta,
    const utils::Scalar& lambda,
    const utils::Scalar& offset,
    const utils::Scalar& r,
    const utils::Scalar& qopt,
    unsigned int dofIdx,
    const utils::String &jointName) :
    actuator::Actuator(direction, dofIdx, jointName),
    m_theta(std::make_shared<utils::Scalar>(theta)),
    m_lambda(std::make_shared<utils::Scalar>(lambda)),
    m_offset(std::make_shared<utils::Scalar>(offset)),
    m_r(std::make_shared<utils::Scalar>(r)),
    m_qopt(std::make_shared<utils::Scalar>(qopt))
{
    setType();
}

actuator::ActuatorSigmoidGauss3p::~ActuatorSigmoidGauss3p()
{

}

actuator::ActuatorSigmoidGauss3p
actuator::ActuatorSigmoidGauss3p::DeepCopy() const
{
    actuator::ActuatorSigmoidGauss3p copy;
    copy.DeepCopy(*this);
    return copy;
}

void actuator::ActuatorSigmoidGauss3p::DeepCopy(
    const actuator::ActuatorSigmoidGauss3p &other)
{
    actuator::Actuator::DeepCopy(other);
    *m_theta = *other.m_theta;
    *m_lambda = *other.m_lambda;
    *m_offset = *other.m_offset;
    *m_r = *other.m_r;
    *m_qopt = *other.m_qopt;
}

utils::Scalar actuator::ActuatorSigmoidGauss3p::torqueMax()
{
    utils::Error::raise(
        "torqueMax for ActuatorSigmoidGauss3p must be called with Q and Qdot");
}

utils::Scalar actuator::ActuatorSigmoidGauss3p::torqueMax(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    utils::Scalar pos(Q[*m_dofIdx] * 180/M_PI);
    utils::Scalar speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Getting Tmax of Gauss3p from Sigmoid
    utils::Scalar Tmax(*m_theta / (1 + exp(*m_lambda * speed)) + *m_offset);

    // Calculation of the max torque
    return Tmax * exp(-(*m_qopt - pos) * (*m_qopt - pos) / (2 * *m_r * *m_r));
}

void actuator::ActuatorSigmoidGauss3p::setType()
{
    *m_type = actuator::TYPE::SIGMOIDGAUSS3P;
}
