#define BIORBD_API_EXPORTS
#include "RigidBody/IMU.h"

#include "Utils/String.h"

biorbd::rigidbody::IMU::IMU(
        bool isTechnical,
        bool isAnatomical) :
    biorbd::utils::RotoTransNode(),
    m_technical(isTechnical),
    m_anatomical(isAnatomical)
{

}
biorbd::rigidbody::IMU::IMU(
        const biorbd::utils::RotoTransNode &RotoTrans,
        bool isTechnical,
        bool isAnatomical) :
    biorbd::utils::RotoTransNode(RotoTrans),
    m_technical(isTechnical),
    m_anatomical(isAnatomical)
{

}

biorbd::rigidbody::IMU biorbd::rigidbody::IMU::DeepCopy() const
{
    return biorbd::rigidbody::IMU(this->biorbd::utils::RotoTransNode::DeepCopy(), this->isTechnical(), this->isAnatomical());
}

bool biorbd::rigidbody::IMU::isAnatomical() const
{
    return m_anatomical;
}

bool biorbd::rigidbody::IMU::isTechnical() const
{
    return m_technical;
}
