#define BIORBD_API_EXPORTS
#include "InternalForces/ViaPoint.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

internalforce::ViaPoint::ViaPoint() :
    utils::Vector3d()
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internalforce::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internalforce::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d(x, y, z, name, parentName)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internalforce::ViaPoint::ViaPoint(
    const utils::Vector3d &other) :
    utils::Vector3d(other)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

internalforce::ViaPoint::ViaPoint(const internalforce::ViaPoint &other) :
    utils::Vector3d(other)
{

}

internalforce::ViaPoint internalforce::ViaPoint::DeepCopy() const
{
    internalforce::ViaPoint copy;
    copy.DeepCopy(*this);
    return copy;
}

void internalforce::ViaPoint::DeepCopy(const internalforce::ViaPoint &other)
{
    utils::Vector3d::DeepCopy(other);
}
