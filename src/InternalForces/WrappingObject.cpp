#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingObject.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

internalforce::WrappingObject::WrappingObject() :
    utils::Vector3d (),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

internalforce::WrappingObject::WrappingObject(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

internalforce::WrappingObject::WrappingObject(
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

internalforce::WrappingObject::WrappingObject(
    const utils::Vector3d &other) :
    utils::Vector3d (other)
{
    try {
        internalforce::WrappingObject& otherWrap(
            const_cast<internalforce::WrappingObject&>(
                dynamic_cast<const internalforce::WrappingObject&>(other)));
        m_RT = otherWrap.m_RT;
    } catch(const std::bad_cast& e) {
        m_RT = std::make_shared<utils::RotoTrans>();
    }
}

internalforce::WrappingObject::WrappingObject(
    const utils::Vector3d &other,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d (other, name, parentName),
    m_RT(std::make_shared<utils::RotoTrans>())
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_OBJECT;
}

void internalforce::WrappingObject::DeepCopy(
    const internalforce::WrappingObject &other)
{
    utils::Vector3d::DeepCopy(other);
    *m_RT = *other.m_RT;
}

const utils::RotoTrans &internalforce::WrappingObject::RT() const
{
    return *m_RT;
}
