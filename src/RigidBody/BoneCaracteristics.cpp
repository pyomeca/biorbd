#define BIORBD_API_EXPORTS
#include "RigidBody/BoneCaracteristics.h"

#include "Utils/Node.h"
#include "RigidBody/Patch.h"

biorbd::rigidbody::Caracteristics::Caracteristics() :
    Body(),
    m_length(0),
    m_mesh(biorbd::rigidbody::Mesh())
{
}
biorbd::rigidbody::Caracteristics::Caracteristics(
        double mass,
        const biorbd::utils::Node &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia,
        const biorbd::rigidbody::Mesh &mesh) :
    Body(mass, com, inertia),
    m_length(0),
    m_mesh(mesh)
{
}

const biorbd::rigidbody::Mesh &biorbd::rigidbody::Caracteristics::mesh() const {
    return m_mesh;
}

const Eigen::Matrix3d &biorbd::rigidbody::Caracteristics::inertia() const {
    return mInertia;
}

biorbd::rigidbody::Caracteristics::~Caracteristics()
{
    //dtor
}

double biorbd::rigidbody::Caracteristics::length() const {
    return m_length;
}

double biorbd::rigidbody::Caracteristics::mass() const {
    return mMass;
}

void biorbd::rigidbody::Caracteristics::setLength(double val) {
    m_length = val;
}
