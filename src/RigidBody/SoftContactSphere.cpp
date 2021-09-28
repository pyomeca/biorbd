#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContactSphere.h"

#include "Utils/String.h"

using namespace BIORBD_NAMESPACE;

rigidbody::SoftContactSphere::SoftContactSphere():
    rigidbody::SoftContactNode(),
    m_radius(std::make_shared<utils::Scalar>(-1)),
    m_stiffness(std::make_shared<utils::Scalar>(-1)),
    m_damping(std::make_shared<utils::Scalar>(-1)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
{
    setType();
}

rigidbody::SoftContactSphere::SoftContactSphere(const SoftContactNode & other):
    rigidbody::SoftContactNode(other),
    m_radius(std::make_shared<utils::Scalar>(-1)),
    m_stiffness(std::make_shared<utils::Scalar>(-1)),
    m_damping(std::make_shared<utils::Scalar>(-1)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
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
    m_damping(std::make_shared<utils::Scalar>(damping)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
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
    m_damping(std::make_shared<utils::Scalar>(damping)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
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
    m_damping(std::make_shared<utils::Scalar>(damping)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
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
    m_damping(std::make_shared<utils::Scalar>(damping)),
    m_muStatic(std::make_shared<utils::Scalar>(0.8)),
    m_muDynamic(std::make_shared<utils::Scalar>(0.7)),
    m_muViscous(std::make_shared<utils::Scalar>(0.5)),
    m_transitionVelocity(std::make_shared<utils::Scalar>(0.01))
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
    *m_muStatic = *other.m_muStatic;
    *m_muDynamic = *other.m_muDynamic;
    *m_muViscous = *other.m_muViscous;
    *m_transitionVelocity = *other.m_transitionVelocity;
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

void rigidbody::SoftContactSphere::setMuStatic(
        const utils::Scalar &muStatic)
{
    *m_muStatic = muStatic;
}

utils::Scalar rigidbody::SoftContactSphere::muStatic() const
{
    return *m_muStatic;
}

void rigidbody::SoftContactSphere::setMuDynamic(
        const utils::Scalar &muDynamic)
{
    *m_muDynamic = muDynamic;
}

utils::Scalar rigidbody::SoftContactSphere::muDynamic() const
{
    return *m_muDynamic;
}

void rigidbody::SoftContactSphere::setMuViscous(
        const utils::Scalar &muViscous)
{
    *m_muViscous = muViscous;
}

utils::Scalar rigidbody::SoftContactSphere::muViscous() const
{
    return *m_muViscous;
}

void rigidbody::SoftContactSphere::setTransitionVelocity(
        const utils::Scalar &transitionVelocity)
{
    *m_transitionVelocity = transitionVelocity;
}

utils::Scalar rigidbody::SoftContactSphere::transitionVelocity() const
{
    return *m_transitionVelocity;
}

utils::Vector3d rigidbody::SoftContactSphere::computeForce(
        const utils::Vector3d& x,
        const utils::Vector3d& dx) const
{

    // Indentation detection with the ground
    const utils::Vector3d& plane(m_contactPlane->first);
    const utils::Vector3d& normal(m_contactPlane->second);

    // Decomposition into normal and tangent velocities
    utils::Scalar normalVelocity = dx.dot(normal);
    utils::Vector3d tangentVelocity = dx - normalVelocity * normal;

    // Penetration of the sphere in the plane
    utils::Scalar delta = -((x - plane).dot(normal) - *m_radius);
    utils::Scalar deltaDot = -normalVelocity;

    // Compute the smoothing factor
    utils::Scalar eps(1e-16);
    utils::Scalar bv(50);
    utils::Scalar bd(300);
    utils::Scalar fslope = (0.5 + 0.5 * tanh(bd * delta) + eps)
            * (0.5 + 0.5 * tanh(bv * (deltaDot + 2. / 3. / *m_damping ) + eps));

    // Force factor on normal from Hertz's model
    // sqrt and **2 to get positive values in case of negative penetration
    utils::Scalar deltaSquaredRooted(sqrtf(delta * delta));
    utils::Scalar forceFactor = 4. / 3. * *m_stiffness * sqrtf(*m_radius)
            * sqrtf(deltaSquaredRooted * deltaSquaredRooted * deltaSquaredRooted);

    // Hunt-Crossley' model
    utils::Scalar fHC = forceFactor * (1. + 1.5 * *m_damping * deltaDot);
    utils::Scalar normalForce = fHC * fslope;

    utils::Scalar tangentVelocityNorm(sqrtf(tangentVelocity.squaredNorm() + 1e-5));
    utils::Scalar frictionVelocity = tangentVelocityNorm / *m_transitionVelocity;

    // from Peter Brown 2017
    utils::Scalar forceFriction = normalForce * *m_muDynamic * tanh(4. * frictionVelocity)
            + normalForce * (*m_muStatic - *m_muDynamic) * frictionVelocity
                   / ((0.25 * frictionVelocity * frictionVelocity + 0.75) * (0.25 * frictionVelocity * frictionVelocity + 0.75))
            + normalForce * *m_muViscous * tangentVelocityNorm;

    // Total Force
    return normalForce * normal + forceFriction * -tangentVelocity / tangentVelocityNorm;
}

utils::Vector3d rigidbody::SoftContactSphere::applicationPoint(
        const Vector3d &x) const
{
    const utils::Vector3d& plane(m_contactPlane->first);
    const utils::Vector3d& normal(m_contactPlane->second);
    utils::Scalar delta = -((x - plane).dot(normal) - *m_radius);

    return (x - plane) - delta * normal;
}

void rigidbody::SoftContactSphere::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::SOFT_CONTACT_SPHERE;
}
