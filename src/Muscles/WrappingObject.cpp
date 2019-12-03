#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

biorbd::muscles::WrappingObject::WrappingObject() :
    biorbd::utils::Vector3d (),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
        double x,
        double y,
        double z) :
    biorbd::utils::Vector3d(x, y, z),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Vector3d(x, y, z, name, parentName),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(const biorbd::utils::Vector3d &other) :
    biorbd::utils::Vector3d (other),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
        const Eigen::Vector3d &other,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Vector3d (other, name, parentName)
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

void biorbd::muscles::WrappingObject::DeepCopy(const biorbd::muscles::WrappingObject &other)
{
    biorbd::utils::Vector3d::DeepCopy(other);
    *m_RT = *other.m_RT;
}

const biorbd::utils::RotoTrans &biorbd::muscles::WrappingObject::RT() const
{
    return *m_RT;
}
