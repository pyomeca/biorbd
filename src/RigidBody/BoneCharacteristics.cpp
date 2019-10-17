#define BIORBD_API_EXPORTS
#include "RigidBody/BoneCharacteristics.h"

#include "Utils/Node3d.h"
#include "RigidBody/Patch.h"
#include "RigidBody/BoneMesh.h"

biorbd::rigidbody::BoneCharacteristics::BoneCharacteristics() :
    Body(),
    m_length(std::make_shared<double>(0)),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>())
{

}
biorbd::rigidbody::BoneCharacteristics::BoneCharacteristics(
        double mass,
        const biorbd::utils::Node3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia) :
    Body(mass, com, inertia),
    m_length(std::make_shared<double>(0)),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>())
{

}
biorbd::rigidbody::BoneCharacteristics::BoneCharacteristics(
        double mass,
        const biorbd::utils::Node3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia,
        const biorbd::rigidbody::BoneMesh &mesh) :
    Body(mass, com, inertia),
    m_length(std::make_shared<double>(0)),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>(mesh))
{

}

biorbd::rigidbody::BoneCharacteristics biorbd::rigidbody::BoneCharacteristics::DeepCopy() const
{
    biorbd::rigidbody::BoneCharacteristics copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::BoneCharacteristics::DeepCopy(const BoneCharacteristics &other)
{
    static_cast<RigidBodyDynamics::Body&>(*this) = other;
    *m_length = *other.m_length;
    *m_mesh = other.m_mesh->DeepCopy();
}

const biorbd::rigidbody::BoneMesh &biorbd::rigidbody::BoneCharacteristics::mesh() const
{
    return *m_mesh;
}

const Eigen::Matrix3d &biorbd::rigidbody::BoneCharacteristics::inertia() const
{
    return mInertia;
}

double biorbd::rigidbody::BoneCharacteristics::length() const
{
    return *m_length;
}

double biorbd::rigidbody::BoneCharacteristics::mass() const
{
    return mMass;
}

void biorbd::rigidbody::BoneCharacteristics::setLength(double val)
{
    *m_length = val;
}
