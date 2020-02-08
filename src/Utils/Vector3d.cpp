#define BIORBD_API_EXPORTS
#include "Utils/Vector3d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

biorbd::utils::Vector3d::Vector3d() :
    RigidBodyDynamics::Math::Vector3d (),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(
        RigidBodyDynamics::Math::Scalar x,
        RigidBodyDynamics::Math::Scalar y,
        RigidBodyDynamics::Math::Scalar z) :
    RigidBodyDynamics::Math::Vector3d (x, y, z),
    biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(
        const RigidBodyDynamics::Math::Vector4d &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]), biorbd::utils::Node ()
{
    setType();
}

#ifdef BIORBD_USE_CASADI_MATH
biorbd::utils::Vector3d::Vector3d(
        const RigidBodyDynamics::Math::Vector3d &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]), biorbd::utils::Node ()
{
    setType();
}

biorbd::utils::Vector3d::Vector3d(
        const RigidBodyDynamics::Math::VectorNd &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]), biorbd::utils::Node ()
{
    setType();
}
#endif

biorbd::utils::Vector3d::Vector3d(
        double x,
        double y,
        double z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
    RigidBodyDynamics::Math::Vector3d (x, y, z),
    biorbd::utils::Node (name, parentName)
{
    setType();
}

biorbd::utils::Vector3d biorbd::utils::Vector3d::DeepCopy() const
{
    biorbd::utils::Vector3d copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Vector3d::DeepCopy(const Vector3d &other)
{
    *this = other;
    biorbd::utils::Node::DeepCopy(other);
}

biorbd::utils::Vector3d biorbd::utils::Vector3d::applyRT(const biorbd::utils::RotoTrans &rt) const
{
    RigidBodyDynamics::Math::Vector4d v;
    v.block(0, 0, 3, 1) = *this;
    v[3] = 1;
    return static_cast<RigidBodyDynamics::Math::VectorNd>((rt * v).block(0, 0, 3, 1));
}

void biorbd::utils::Vector3d::applyRT(const biorbd::utils::RotoTrans &rt){
    RigidBodyDynamics::Math::Vector4d v;
    v.block(0, 0, 3, 1) = *this;
    v[3] = 1;
    *this = (rt * v).block(0, 0, 3, 1);
}

biorbd::utils::Vector3d &biorbd::utils::Vector3d::operator=(const RigidBodyDynamics::Math::Vector4d &v){
    this->RigidBodyDynamics::Math::Vector3d::operator=(biorbd::utils::Vector3d(v));
    return *this;
}

void biorbd::utils::Vector3d::setType()
{
    *m_typeOfNode = biorbd::utils::VECTOR3D;
}
