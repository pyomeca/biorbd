#define BIORBD_API_EXPORTS
#include "RigidBody/BoneCaracteristics.h"

#include "Utils/Node3d.h"
#include "RigidBody/Patch.h"
#include "RigidBody/BoneMesh.h"

biorbd::rigidbody::Caracteristics::Caracteristics() :
    Body(),
    m_length(0),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>())
{

}
biorbd::rigidbody::Caracteristics::Caracteristics(
        double mass,
        const biorbd::utils::Node3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia) :
    Body(mass, com, inertia),
    m_length(0),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>())
{

}
biorbd::rigidbody::Caracteristics::Caracteristics(
        double mass,
        const biorbd::utils::Node3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia,
        const biorbd::rigidbody::BoneMesh &mesh) :
    Body(mass, com, inertia),
    m_length(0),
    m_mesh(std::make_shared<biorbd::rigidbody::BoneMesh>(mesh))
{

}

const biorbd::rigidbody::BoneMesh &biorbd::rigidbody::Caracteristics::mesh() const
{
    return *m_mesh;
}

const Eigen::Matrix3d &biorbd::rigidbody::Caracteristics::inertia() const
{
    return mInertia;
}

double biorbd::rigidbody::Caracteristics::length() const
{
    return m_length;
}

double biorbd::rigidbody::Caracteristics::mass() const
{
    return mMass;
}

void biorbd::rigidbody::Caracteristics::setLength(double val)
{
    m_length = val;
}
