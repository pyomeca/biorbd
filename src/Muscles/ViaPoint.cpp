#define BIORBD_API_EXPORTS
#include "Muscles/ViaPoint.h"

#include "Utils/String.h"

biorbd::muscles::ViaPoint::ViaPoint() :
    biorbd::utils::Node3d()
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::VIA_POINT;
}

biorbd::muscles::ViaPoint::ViaPoint(
        double x,
        double y,
        double z) :
    biorbd::utils::Node3d(x, y, z)
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::VIA_POINT;
}

biorbd::muscles::ViaPoint::ViaPoint(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node3d(x, y, z, name, parentName)
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::VIA_POINT;
}

biorbd::muscles::ViaPoint::ViaPoint(
        const biorbd::utils::Node3d &other) :
    biorbd::utils::Node3d(other)
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::VIA_POINT;
}

biorbd::muscles::ViaPoint::ViaPoint(const biorbd::muscles::ViaPoint &other) :
    biorbd::utils::Node3d(other)
{

}

biorbd::muscles::ViaPoint biorbd::muscles::ViaPoint::DeepCopy() const
{
    biorbd::muscles::ViaPoint copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::ViaPoint::DeepCopy(const biorbd::muscles::ViaPoint &other)
{
    biorbd::utils::Node3d::DeepCopy(other);
}
