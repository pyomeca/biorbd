#define BIORBD_API_EXPORTS
#include "Muscles/WrappingSphere.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

muscles::WrappingSphere::WrappingSphere() :
    muscles::WrappingObject (),
    m_dia(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

muscles::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter) :
    muscles::WrappingObject (x, y, z),
    m_dia(std::make_shared<utils::Scalar>(diameter))

{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

muscles::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter,
    const utils::String &name,
    const utils::String &parentName) :
    muscles::WrappingObject (x, y, z, name, parentName),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

muscles::WrappingSphere::WrappingSphere(
    const utils::Vector3d &v,
    const utils::Scalar& diameter) :
    muscles::WrappingObject(v),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

muscles::WrappingSphere muscles::WrappingSphere::DeepCopy()
const
{
    muscles::WrappingSphere copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::WrappingSphere::DeepCopy(const
        muscles::WrappingSphere &other)
{
    muscles::WrappingObject::DeepCopy(other);
    *m_dia = *other.m_dia;
}

const utils::RotoTrans& muscles::WrappingSphere::RT(
    biorbd::BIORBD_MATH_NAMESPACE::rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    bool)
{
    return *m_RT;
}

void muscles::WrappingSphere::setDiameter(
    const utils::Scalar& val)
{
    *m_dia = val;
}

const utils::Scalar& muscles::WrappingSphere::diameter() const
{
    return *m_dia;
}
