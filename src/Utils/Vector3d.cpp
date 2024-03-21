#define BIORBD_API_EXPORTS
#include "Utils/Vector3d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;

utils::Vector3d::Vector3d() :
    RigidBodyDynamics::Math::Vector3d (RigidBodyDynamics::Math::Vector3d::Zero()),
    utils::Node ()
{
    setType();
}

utils::Vector3d::Vector3d(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z) :
    RigidBodyDynamics::Math::Vector3d (x, y, z),
    utils::Node ()
{
    setType();
}

utils::Vector3d::Vector3d(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::Scalar& z,
    const utils::String &name,
    const utils::String &parentName) :
    RigidBodyDynamics::Math::Vector3d (x, y, z),
    utils::Node (name, parentName)
{
    setType();
}

utils::Vector3d::Vector3d(
    const utils::Vector3d vec,
    const utils::String &name,
    const utils::String &parentName) :
    RigidBodyDynamics::Math::Vector3d (vec),
    Node(name, parentName)
{

}

utils::Vector3d::Vector3d(
    const RigidBodyDynamics::Math::Vector3d &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]),
    utils::Node ()
{
    setType();
}

utils::Vector3d::Vector3d(
    const RigidBodyDynamics::Math::VectorNd &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]),
    utils::Node ()
{
    setType();
}

utils::Vector3d::Vector3d(
    const RigidBodyDynamics::Math::Vector4d &other) :
    RigidBodyDynamics::Math::Vector3d(other[0], other[1], other[2]),
    utils::Node ()
{
    setType();
}

#ifdef BIORBD_USE_CASADI_MATH

utils::Vector3d::Vector3d(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other) :
    RigidBodyDynamics::Math::Vector3d(other),
    utils::Node ()
{
    setType();
}

#endif


utils::Vector3d utils::Vector3d::DeepCopy() const
{
    utils::Vector3d copy;
    copy.DeepCopy(*this);
    return copy;
}

void utils::Vector3d::DeepCopy(const Vector3d &other)
{
    this->RigidBodyDynamics::Math::Vector3d::operator=(other);
    utils::Node::DeepCopy(other);
}

utils::Vector3d utils::Vector3d::applyRT(
    const utils::RotoTrans &rt) const
{
    RigidBodyDynamics::Math::Vector4d v;
    v.block(0, 0, 3, 1) = *this;
    v[3] = 1;
    return (rt * v).block(0, 0, 3, 1);
}

void utils::Vector3d::applyRT(
    const utils::RotoTrans &rt)
{
    RigidBodyDynamics::Math::Vector4d v;
    v.block(0, 0, 3, 1) = *this;
    v[3] = 1;
    setPosition((rt * v).block(0, 0, 3, 1));
}

void utils::Vector3d::setPosition(
    const utils::Vector3d& v)
{
#ifdef BIORBD_USE_CASADI_MATH
    (*this)(0) = v(0);
    (*this)(1) = v(1);
    (*this)(2) = v(2);
#else
    *this = v;
#endif
}

void utils::Vector3d::setType()
{
    *m_typeOfNode = utils::VECTOR3D;
}

#ifdef BIORBD_USE_CASADI_MATH
utils::Scalar utils::Vector3d::x() const
{
    return (*this)[0];
}

utils::Scalar utils::Vector3d::y() const
{
    return (*this)[1];
}

utils::Scalar utils::Vector3d::z() const
{
    return (*this)[2];
}

void utils::Vector3d::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    *this = utils::Vector3d(other);
}

void utils::Vector3d::operator=(
    const RigidBodyDynamics::Math::Vector4d& other)
{
    *this = utils::Vector3d(other);
}
#endif
