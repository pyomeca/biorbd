#define BIORBD_API_EXPORTS
#include "RigidBody/SegmentCharacteristics.h"

#include "Utils/Scalar.h"
#include "Utils/Vector3d.h"
#include "Utils/Matrix3d.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/Mesh.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SegmentCharacteristics::SegmentCharacteristics() :
    Body(),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_mesh(std::make_shared<rigidbody::Mesh>())
{

}
rigidbody::SegmentCharacteristics::SegmentCharacteristics(
    const utils::Scalar &mass,
    const utils::Vector3d &com,
    const utils::Matrix3d &inertia) :
    Body(mass, com, inertia),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_mesh(std::make_shared<rigidbody::Mesh>())
{

}
rigidbody::SegmentCharacteristics::SegmentCharacteristics(
    const utils::Scalar &mass,
    const utils::Vector3d &com,
    const utils::Matrix3d &inertia,
    const rigidbody::Mesh &mesh) :
    Body(mass, com, inertia),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_mesh(std::make_shared<rigidbody::Mesh>(mesh))
{

}

rigidbody::SegmentCharacteristics
rigidbody::SegmentCharacteristics::DeepCopy() const
{
    rigidbody::SegmentCharacteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::SegmentCharacteristics::DeepCopy(
    const SegmentCharacteristics &other)
{
    static_cast<RigidBodyDynamics::Body&>(*this) = other;
    *m_length = *other.m_length;
    *m_mesh = other.m_mesh->DeepCopy();
}

void rigidbody::SegmentCharacteristics::setLength(
    const utils::Scalar& val)
{
    *m_length = val;
}

utils::Scalar rigidbody::SegmentCharacteristics::length() const
{
    return *m_length;
}

void rigidbody::SegmentCharacteristics::setMass(
    const utils::Scalar &newMass)
{
    mMass = newMass;
}

utils::Scalar rigidbody::SegmentCharacteristics::mass() const
{
    return mMass;
}

utils::Vector3d rigidbody::SegmentCharacteristics::CoM() const
{
    return mCenterOfMass;
}

void rigidbody::SegmentCharacteristics::setCoM(
    const utils::Vector3d &com)
{
    mCenterOfMass = com;
}

const rigidbody::Mesh &rigidbody::SegmentCharacteristics::mesh()
const
{
    return *m_mesh;
}

const utils::Matrix3d rigidbody::SegmentCharacteristics::inertia() const
{
    return mInertia;
}
