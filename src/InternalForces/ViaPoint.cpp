#define BIORBD_API_EXPORTS
#include "InternalForces/ViaPoint.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internal_forces::ViaPoint::ViaPoint() :
    utils::Vector3d()
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internal_forces::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internal_forces::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d(x, y, z, name, parentName)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internal_forces::ViaPoint::ViaPoint(
    const utils::Vector3d &other) :
    utils::Vector3d(other)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internal_forces::ViaPoint::ViaPoint(const internal_forces::ViaPoint &other) :
    utils::Vector3d(other)
{

}

internal_forces::ViaPoint internal_forces::ViaPoint::DeepCopy() const
{
    internal_forces::ViaPoint copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ViaPoint::DeepCopy(const internal_forces::ViaPoint &other)
{
    utils::Vector3d::DeepCopy(other);
}
