#define BIORBD_API_EXPORTS
#include "RigidBody/IMU.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

rigidbody::IMU::IMU(
    bool isTechnical,
    bool isAnatomical) :
    utils::RotoTransNode(),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}

rigidbody::IMU::IMU(
    const utils::RotoTransNode &RotoTrans,
    bool isTechnical,
    bool isAnatomical) :
    utils::RotoTransNode(RotoTrans),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}

#ifdef BIORBD_USE_CASADI_MATH

rigidbody::IMU::IMU(
    const rigidbody::IMU &imu) :
    utils::RotoTransNode (imu),
    m_technical(std::make_shared<bool>(*imu.m_technical)),
    m_anatomical(std::make_shared<bool>(*imu.m_anatomical))
{

}

rigidbody::IMU rigidbody::IMU::operator*(
    const rigidbody::IMU &other) const
{
    return rigidbody::IMU(
               utils::RotoTransNode(
                   this->utils::RotoTransNode::operator*(other),
                   this->utils::Node::name(),
                   this->parent()),
               this->isTechnical() && other.isTechnical(),
               this->isAnatomical() && other.isAnatomical());
}

#endif

rigidbody::IMU rigidbody::IMU::DeepCopy() const
{
    rigidbody::IMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::IMU::DeepCopy(const IMU &other)
{
    utils::RotoTransNode::DeepCopy(other);
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
}

bool rigidbody::IMU::isAnatomical() const
{
    return *m_anatomical;
}

bool rigidbody::IMU::isTechnical() const
{
    return *m_technical;
}
