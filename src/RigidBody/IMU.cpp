#define BIORBD_API_EXPORTS
#include "RigidBody/IMU.h"

biorbd::rigidbody::IMU::IMU(
        const biorbd::utils::Attitude &v,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        bool tech,
        bool ana,
        int id) :
    biorbd::utils::NodeAttitude(v, name, parentName),
    m_technical(tech),
    m_anatomical(ana),
    m_id(id)
{
    //ctor
}

biorbd::rigidbody::IMU::~IMU()
{
    //dtor
}




bool biorbd::rigidbody::IMU::isAnatomical() const
{
    return m_anatomical;
}



bool biorbd::rigidbody::IMU::isTechnical() const
{
    return m_technical;
}


int biorbd::rigidbody::IMU::parentId() const
{
    return m_id;
}
