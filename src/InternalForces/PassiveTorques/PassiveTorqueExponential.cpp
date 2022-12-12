#define BIORBD_API_EXPORTS
#include "InternalForces/PassiveTorques/PassiveTorqueExponential.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"

using namespace BIORBD_NAMESPACE;

internal_forces::passive_torques::PassiveTorqueExponential::PassiveTorqueExponential() :
    internal_forces::passive_torques::PassiveTorque(),
    m_k1(std::make_shared<utils::Scalar>(0)),
    m_k2(std::make_shared<utils::Scalar>(0)),
    m_b1(std::make_shared<utils::Scalar>(0)),
    m_b2(std::make_shared<utils::Scalar>(0)),
    m_qMid(std::make_shared<utils::Scalar>(0)),
    m_tauEq(std::make_shared<utils::Scalar>(0)),
    m_pBeta(std::make_shared<utils::Scalar>(0)),
    m_wMax(std::make_shared<utils::Scalar>(0)),
    m_sV(std::make_shared<utils::Scalar>(0)),
    m_deltaP(std::make_shared<utils::Scalar>(0))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueExponential::PassiveTorqueExponential(
    const internal_forces::passive_torques::PassiveTorqueExponential &other) :
    internal_forces::passive_torques::PassiveTorque(other),
    m_k1(other.m_k1),
    m_k2(other.m_k2),
    m_b1(other.m_b1),
    m_b2(other.m_b2),
    m_qMid(other.m_qMid),
    m_tauEq(other.m_tauEq),
    m_pBeta(other.m_pBeta),
    m_wMax(other.m_wMax),
    m_sV(other.m_sV),
    m_deltaP(other.m_deltaP)
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueExponential::PassiveTorqueExponential(
    const utils::Scalar& k1,
    const utils::Scalar& k2,
    const utils::Scalar& b1,
    const utils::Scalar& b2,
    const utils::Scalar& qMid,
    const utils::Scalar& tauEq,
    const utils::Scalar& pBeta,
    const utils::Scalar& wMax,
    const utils::Scalar& sV,
    const utils::Scalar& deltaP,
    unsigned int dofIdx) :
    internal_forces::passive_torques::PassiveTorque(dofIdx),
    m_k1(std::make_shared<utils::Scalar>(k1)),
    m_k2(std::make_shared<utils::Scalar>(k2)),
    m_b1(std::make_shared<utils::Scalar>(b1)),
    m_b2(std::make_shared<utils::Scalar>(b2)),
    m_qMid(std::make_shared<utils::Scalar>(qMid)),
    m_tauEq(std::make_shared<utils::Scalar>(tauEq)),
    m_pBeta(std::make_shared<utils::Scalar>(pBeta)),
    m_wMax(std::make_shared<utils::Scalar>(wMax)),
    m_sV(std::make_shared<utils::Scalar>(sV)),
    m_deltaP(std::make_shared<utils::Scalar>(deltaP))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueExponential::PassiveTorqueExponential(
    const utils::Scalar& k1,
    const utils::Scalar& k2,
    const utils::Scalar& b1,
    const utils::Scalar& b2,
    const utils::Scalar& qMid,
    const utils::Scalar& tauEq,
    const utils::Scalar& pBeta,
    const utils::Scalar& wMax,
    const utils::Scalar& sV,
    const utils::Scalar& deltaP,
    unsigned int dofIdx,
    const utils::String &jointName) :
    internal_forces::passive_torques::PassiveTorque(dofIdx, jointName),
    m_k1(std::make_shared<utils::Scalar>(k1)),
    m_k2(std::make_shared<utils::Scalar>(k2)),
    m_b1(std::make_shared<utils::Scalar>(b1)),
    m_b2(std::make_shared<utils::Scalar>(b2)),
    m_qMid(std::make_shared<utils::Scalar>(qMid)),
    m_tauEq(std::make_shared<utils::Scalar>(tauEq)),
    m_pBeta(std::make_shared<utils::Scalar>(pBeta)),
    m_wMax(std::make_shared<utils::Scalar>(wMax)),
    m_sV(std::make_shared<utils::Scalar>(sV)),
    m_deltaP(std::make_shared<utils::Scalar>(deltaP))
{
    setType();
}

internal_forces::passive_torques::PassiveTorqueExponential::~PassiveTorqueExponential()
{

}

internal_forces::passive_torques::PassiveTorqueExponential internal_forces::passive_torques::PassiveTorqueExponential::DeepCopy()
const
{
    internal_forces::passive_torques::PassiveTorqueExponential copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::passive_torques::PassiveTorqueExponential::DeepCopy(const
        internal_forces::passive_torques::PassiveTorqueExponential &other)
{
    internal_forces::passive_torques::PassiveTorque::DeepCopy(other);
    *m_k1 = *other.m_k1;
    *m_k2 = *other.m_k2;
    *m_b1 = *other.m_b1;
    *m_b2 = *other.m_b2;
    *m_qMid = *other.m_qMid;
    *m_tauEq = *other.m_tauEq;
    *m_pBeta = *other.m_pBeta;
    *m_wMax = *other.m_wMax;
    *m_sV = *other.m_sV;
    *m_deltaP = *other.m_deltaP;
}

utils::Scalar internal_forces::passive_torques::PassiveTorqueExponential::passiveTorque()
{
    utils::Error::raise("passiveTorque for PassiveTorqueExponential must be called with Q");
}


utils::Scalar internal_forces::passive_torques::PassiveTorqueExponential::passiveTorque(
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedCoordinates &Qdot) const
{
    return (*m_b1 * exp(*m_k1 * (Q[*m_dofIdx] - *m_qMid)) + *m_b2 * exp(*m_k2 * (Q[*m_dofIdx] - *m_qMid)))
            * (1 - *m_pBeta * (Qdot[*m_dofIdx] /
            (*m_sV * *m_wMax))) * (Q[*m_dofIdx] - *m_deltaP) + *m_tauEq;
}

void internal_forces::passive_torques::PassiveTorqueExponential::setType()
{
    *m_type = internal_forces::passive_torques::TORQUE_TYPE::TORQUE_EXPONENTIAL;
}
