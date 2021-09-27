#define BIORBD_API_EXPORTS
#include "RigidBody/SoftContactSphere.h"

#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

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

RigidBodyDynamics::Math::SpatialVector rigidbody::SoftContactSphere::computeForce(
        const rigidbody::GeneralizedCoordinates& Q,
        const rigidbody::GeneralizedVelocity& QDot) const
{
    RigidBodyDynamics::Math::SpatialVector f(0, 0, 0, 0, 0, 0);

    // Contact Position and velocity
    x = rigidbody::SoftContacts::softContact(Q)
    dx = rigidbody::SoftContacts::softContactVelocity(Q, QDot)

    // Indentation detection with the ground
    RigidBodyDynamics::Math::Vector3d grd(0,0,0);
    RigidBodyDynamics::Math::Vector3d n_vec(0,0,1); // normal

    // Decomposition des vitesses
    vn = dx.dot(n_vec)
    vt = dx - vn * n_vec

    // Indentation
    delta = - ((x - grd).dot(n_vec) - rigidbody::SoftContactSphere::radius())
    deltadot = vn

    // location of the normal force
    p = (x - grd) - delta * n_vec

    // Smoothed contact
    eps = 1e-16
    bv = 50
    bd = 300
    fslope = (1 / 2 + 1 / 2 * tanh(bd * delta) + eps) * (
                1 / 2 + 1 / 2 * tanh(bv * (deltadot + 2 / 3 / rigidbody::SoftContacts::SoftContactSphere::damping() ) + eps))

    // Normal force
    // Hertz
    fHz = 4 / 3 * rigidbody::SoftContacts::SoftContactSphere::stiffness() * sqrt(rigidbody::SoftContacts::SoftContactSphere::radius()) * sqrt(delta ** 2) ** (3 / 2)  # // sqrt and **2 to get positive values in case of negative indentation
    // Hunt-Crossley
    fHC = fHz * (1 + 3 / 2 * rigidbody::SoftContacts::SoftContactSphere::damping() * deltadot)
    fn = fHc * fslope

    // Friction
    mu_s = 0.8
    mu_d = 0.7
    mu_v = 0.5
    vtrans = 0.1 // from 0.01 to 0.13

    vt_norm = sqrt((vt[0]**2+vt[1]**2+vt[2]**2 + 1e-5))
    vrel = vt_norm / vtrans

    f_fric = fn * mu_d * tanh(4 * vrel) + fn * (mu_s - mu_d) * vrel / (1 / 4 * vrel ** 2 + 3 / 4) ** 2 + fn * mu_v * vt_norm  // from Peter Brown 2017

    // Total Force
    f(3:5) = fn * n_vec + f_fric * -vt / vt_norm

    // Location of the center of mass
    // parent segment idx
    // CoM = center of mass location

    // Bour's formula to the CoM of the segment
    fCoM = f // force
    fp[0:3] = f[0:3] + cross(f[3:], CoM - p) // torques

    return f;
}

void rigidbody::SoftContactSphere::setType()
{
    *m_typeOfNode = utils::NODE_TYPE::SOFT_CONTACT_SPHERE;
}
