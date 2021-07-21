#define BIORBD_API_EXPORTS
#include "RigidBody/SegmentCharacteristics.h"

#include "Utils/Scalar.h"
#include "Utils/Vector3d.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/Mesh.h"

using namespace biorbd::BIORBD_MATH_NAMESPACE;

rigidbody::SegmentCharacteristics::SegmentCharacteristics() :
    Body(),
    m_length(std::make_shared<biorbd::utils::Scalar>(0)),
    m_mesh(std::make_shared<rigidbody::Mesh>())
{

}
rigidbody::SegmentCharacteristics::SegmentCharacteristics(
    const biorbd::utils::Scalar &mass,
    const biorbd::utils::Vector3d &com,
    const RigidBodyDynamics::Math::Matrix3d &inertia) :
    Body(mass, com, inertia),
    m_length(std::make_shared<biorbd::utils::Scalar>(0)),
    m_mesh(std::make_shared<rigidbody::Mesh>())
{

}
rigidbody::SegmentCharacteristics::SegmentCharacteristics(
    const biorbd::utils::Scalar &mass,
    const biorbd::utils::Vector3d &com,
    const RigidBodyDynamics::Math::Matrix3d &inertia,
    const rigidbody::Mesh &mesh) :
    Body(mass, com, inertia),
    m_length(std::make_shared<biorbd::utils::Scalar>(0)),
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
    const biorbd::utils::Scalar& val)
{
    *m_length = val;
}

biorbd::utils::Scalar rigidbody::SegmentCharacteristics::length() const
{
    return *m_length;
}

void rigidbody::SegmentCharacteristics::setMass(
    const biorbd::utils::Scalar &newMass)
{
    mMass = newMass;
}

biorbd::utils::Scalar rigidbody::SegmentCharacteristics::mass() const
{
    return mMass;
}

biorbd::utils::Vector3d rigidbody::SegmentCharacteristics::CoM() const
{
    return mCenterOfMass;
}

void rigidbody::SegmentCharacteristics::setCoM(
    const biorbd::utils::Vector3d &com)
{
    mCenterOfMass = com;
}

const rigidbody::Mesh &rigidbody::SegmentCharacteristics::mesh()
const
{
    return *m_mesh;
}

const RigidBodyDynamics::Math::Matrix3d
&rigidbody::SegmentCharacteristics::inertia() const
{
    return mInertia;
}
