#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingSphere.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

internal_forces::WrappingSphere::WrappingSphere() :
    internal_forces::WrappingObject (),
    m_dia(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internal_forces::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter) :
    internal_forces::WrappingObject (x, y, z),
    m_dia(std::make_shared<utils::Scalar>(diameter))

{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internal_forces::WrappingSphere::WrappingSphere(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::Scalar& diameter,
    const utils::String &name,
    const utils::String &parentName) :
    internal_forces::WrappingObject (x, y, z, name, parentName),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internal_forces::WrappingSphere::WrappingSphere(
    const utils::Vector3d &v,
    const utils::Scalar& diameter) :
    internal_forces::WrappingObject(v),
    m_dia(std::make_shared<utils::Scalar>(diameter))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_SPHERE;
}

internal_forces::WrappingSphere internal_forces::WrappingSphere::DeepCopy()
const
{
    internal_forces::WrappingSphere copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::WrappingSphere::DeepCopy(const
        internal_forces::WrappingSphere &other)
{
    internal_forces::WrappingObject::DeepCopy(other);
    *m_dia = *other.m_dia;
}

const utils::RotoTrans& internal_forces::WrappingSphere::RT(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    bool)
{
    return *m_RT;
}

void internal_forces::WrappingSphere::setDiameter(
    const utils::Scalar& val)
{
    *m_dia = val;
}

const utils::Scalar& internal_forces::WrappingSphere::diameter() const
{
    return *m_dia;
}
