#define BIORBD_API_EXPORTS
#include "Muscles/ViaPoint.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

muscles::ViaPoint::ViaPoint() :
    utils::Vector3d()
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

muscles::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    utils::Vector3d(x, y, z)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

muscles::ViaPoint::ViaPoint(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName) :
    utils::Vector3d(x, y, z, name, parentName)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

muscles::ViaPoint::ViaPoint(
    const utils::Vector3d &other) :
    utils::Vector3d(other)
{
    *m_typeOfNode = utils::NODE_TYPE::VIA_POINT;
}

muscles::ViaPoint::ViaPoint(const muscles::ViaPoint &other) :
    utils::Vector3d(other)
{

}

muscles::ViaPoint muscles::ViaPoint::DeepCopy() const
{
    muscles::ViaPoint copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::ViaPoint::DeepCopy(const muscles::ViaPoint &other)
{
    utils::Vector3d::DeepCopy(other);
}
