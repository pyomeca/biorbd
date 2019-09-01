#define BIORBD_API_EXPORTS
#include "RigidBody/IMU.h"

#include "Utils/String.h"

biorbd::rigidbody::IMU::IMU(
        bool isTechnical,
        bool isAnatomical) :
    biorbd::utils::RotoTransNode(),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}
biorbd::rigidbody::IMU::IMU(
        const biorbd::utils::RotoTransNode &RotoTrans,
        bool isTechnical,
        bool isAnatomical) :
    biorbd::utils::RotoTransNode(RotoTrans),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}

biorbd::rigidbody::IMU biorbd::rigidbody::IMU::DeepCopy() const
{
    biorbd::rigidbody::IMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::IMU::DeepCopy(const IMU &other)
{
    biorbd::utils::RotoTransNode::DeepCopy(other);
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
}

bool biorbd::rigidbody::IMU::isAnatomical() const
{
    return *m_anatomical;
}

bool biorbd::rigidbody::IMU::isTechnical() const
{
    return *m_technical;
}
