#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContactSphere.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContactSphere::SoftContactSphere():
    rigidbody::SoftContactNode(),
    m_radius(std::make_shared<utils::Scalar>(-1)),
    m_stiffness(std::make_shared<utils::Scalar>(-1)),
    m_damping(std::make_shared<utils::Scalar>(-1))
{
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(const SoftContactNode & other):
    rigidbody::SoftContactNode(other),
    m_radius(std::make_shared<utils::Scalar>(-1)),
    m_stiffness(std::make_shared<utils::Scalar>(-1)),
    m_damping(std::make_shared<utils::Scalar>(-1))
{
    const rigidbody::SoftContactSphere& tp = dynamic_cast<const SoftContactSphere&>(other);
    *m_radius = *tp.m_radius;
    *m_stiffness = *tp.m_stiffness;
    *m_damping = *tp.m_damping;
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z,
        const utils::Scalar &radius,
        const utils::Scalar &stiffness,
        const utils::Scalar &damping):
    rigidbody::SoftContactNode(x, y, z),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_damping(std::make_shared<utils::Scalar>(damping))
{
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(
        const Vector3d &other,
        const utils::Scalar &radius,
        const utils::Scalar &stiffness,
        const utils::Scalar &damping):
    rigidbody::SoftContactNode(other),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_damping(std::make_shared<utils::Scalar>(damping))
{
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(
        const utils::Scalar &x,
        const utils::Scalar &y,
        const utils::Scalar &z,
        const utils::Scalar &radius,
        const utils::Scalar &stiffness,
        const utils::Scalar &damping,
        const utils::String &name,
        const utils::String &parentName,
        int parentID):
    rigidbody::SoftContactNode(x, y, z, name, parentName, parentID),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_damping(std::make_shared<utils::Scalar>(damping))
{
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(
        const Vector3d &node,
        const utils::Scalar &radius,
        const utils::Scalar &stiffness,
        const utils::Scalar &damping,
        const utils::String &name,
        const utils::String &parentName,
        int parentID):
    rigidbody::SoftContactNode(node, name, parentName, parentID),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_stiffness(std::make_shared<utils::Scalar>(stiffness)),
    m_damping(std::make_shared<utils::Scalar>(damping))
{
    setType();
}

rigidbody::SoftContactSphere rigidbody::SoftContactSphere::DeepCopy() const
{
    rigidbody::SoftContactSphere copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::SoftContactSphere::DeepCopy(const SoftContactSphere &other)
{
    rigidbody::SoftContactNode::DeepCopy(other);
    *m_radius = *other.m_radius;
    *m_stiffness = *other.m_stiffness;
    *m_damping = *other.m_damping;
}

void rigidbody::SoftContactSphere::setRadius(
        const utils::Scalar &radius)
{
    *m_radius = radius;
}

utils::Scalar rigidbody::SoftContactSphere::radius() const
{
    return *m_radius;
}

void rigidbody::SoftContactSphere::setStiffness(
        const utils::Scalar &stiffness)
{
    *m_stiffness = stiffness;
}

utils::Scalar rigidbody::SoftContactSphere::stiffness() const
{
    return *m_stiffness;
}

void rigidbody::SoftContactSphere::setDamping(
        const utils::Scalar &damping)
{
    *m_damping = damping;
}

utils::Scalar rigidbody::SoftContactSphere::damping() const
{
    return *m_damping;
}

void rigidbody::SoftContactSphere::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::SOFT_CONTACT_SPHERE;
}
