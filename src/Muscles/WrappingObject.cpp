#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

biorbd::muscles::WrappingObject::WrappingObject() :
    biorbd::utils::Node3d (),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = "Wrapping";
}

biorbd::muscles::WrappingObject::WrappingObject(
        double x,
        double y,
        double z) :
    biorbd::utils::Node3d(x, y, z),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = "Wrapping";
}

biorbd::muscles::WrappingObject::WrappingObject(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node3d(x, y, z, name, parentName),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = "Wrapping";
}

biorbd::muscles::WrappingObject::WrappingObject(const biorbd::utils::Node3d &other) :
    biorbd::utils::Node3d (other),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = "Wrapping";
}

biorbd::muscles::WrappingObject::WrappingObject(
        const Eigen::Vector3d &other,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    biorbd::utils::Node3d (other, name, parentName)
{
    *m_typeOfNode = "Wrapping";
}
