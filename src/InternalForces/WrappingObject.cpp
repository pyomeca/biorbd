#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingObject.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

internal_forces::WrappingObject::WrappingObject() :
    utils::Vector3d (),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

internal_forces::WrappingObject::WrappingObject(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

internal_forces::WrappingObject::WrappingObject(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d(x, y, z, name, parentName),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

internal_forces::WrappingObject::WrappingObject(
    const utils::Vector3d &other) :
    utils::Vector3d (other)
{
    try {
        internal_forces::WrappingObject& otherWrap(
            const_cast<internal_forces::WrappingObject&>(
                dynamic_cast<const internal_forces::WrappingObject&>(other)));
        m_RT = otherWrap.m_RT;
    } catch(const std::bad_cast&) {
        m_RT = std::make_shared<utils::RotoTrans>();
    }
}

internal_forces::WrappingObject::WrappingObject(
    const utils::Vector3d &other,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d (other, name, parentName),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

void internal_forces::WrappingObject::DeepCopy(
    const internal_forces::WrappingObject &other)
{
    utils::Vector3d::DeepCopy(other);
    *m_RT = *other.m_RT;
}

const utils::RotoTrans &internal_forces::WrappingObject::RT() const
{
    return *m_RT;
}
