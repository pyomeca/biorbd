#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingSphere.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

internalforce::WrappingSphere::WrappingSphere() :
    internalforce::WrappingObject (),
    m_dia(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internalforce::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter) :
    internalforce::WrappingObject (x, y, z),
    m_dia(std::make_shared<utils::Scalar>(diameter))

{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internalforce::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter,
    const utils::String &name,
    const utils::String &parentName) :
    internalforce::WrappingObject (x, y, z, name, parentName),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internalforce::WrappingSphere::WrappingSphere(
    const utils::Vector3d &v,
    const utils::Scalar& diameter) :
    internalforce::WrappingObject(v),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internalforce::WrappingSphere internalforce::WrappingSphere::DeepCopy()
const
{
    internalforce::WrappingSphere copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::WrappingSphere::DeepCopy(const
        internalforce::WrappingSphere &other)
{
    internalforce::WrappingObject::DeepCopy(other);
    *m_dia = *other.m_dia;
}

const utils::RotoTrans& internalforce::WrappingSphere::RT(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    bool)
{
    return *m_RT;
}

void internalforce::WrappingSphere::setDiameter(
    const utils::Scalar& val)
{
    *m_dia = val;
}

const utils::Scalar& internalforce::WrappingSphere::diameter() const
{
    return *m_dia;
}
