#define BIORBD_API_EXPORTS
#include "Utils/Vector2d.h"

#include "Utils/RotoTrans.h"
#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;

utils::Vector2d::Vector2d() :
    RigidBodyDynamics::Math::Vector2d (RigidBodyDynamics::Math::Vector2d::Zero()),
    utils::Node ()
{
    setType();
}

utils::Vector2d::Vector2d(
    const utils::Scalar& x,
    const utils::Scalar& y) :
    RigidBodyDynamics::Math::Vector2d (x, y),
    utils::Node ()
{
    setType();
}

utils::Vector2d::Vector2d(
    const utils::Scalar& x,
    const utils::Scalar& y,
    const utils::String &name,
    const utils::String &parentName) :
    RigidBodyDynamics::Math::Vector2d (x, y),
    utils::Node (name, parentName)
{
    setType();
}

utils::Vector2d::Vector2d(
    const utils::Vector2d vec,
    const utils::String &name,
    const utils::String &parentName) :
    RigidBodyDynamics::Math::Vector2d (vec),
    Node(name, parentName)
{

}

utils::Vector2d::Vector2d(
    const RigidBodyDynamics::Math::Vector2d &other) :
    RigidBodyDynamics::Math::Vector2d(other[0], other[1]),
    utils::Node ()
{
    setType();
}

utils::Vector2d::Vector2d(
    const RigidBodyDynamics::Math::VectorNd &other) :
    RigidBodyDynamics::Math::Vector2d(other[0], other[1]),
    utils::Node ()
{
    setType();
}

#ifdef BIORBD_USE_CASADI_MATH

utils::Vector2d::Vector2d(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other) :
    RigidBodyDynamics::Math::Vector2d(other),
    utils::Node ()
{
    setType();
}

#endif


utils::Vector2d utils::Vector2d::DeepCopy() const
{
    utils::Vector2d copy;
    copy.DeepCopy(*this);
    return copy;
}

void utils::Vector2d::DeepCopy(const Vector2d &other)
{
    this->RigidBodyDynamics::Math::Vector2d::operator=(other);
    utils::Node::DeepCopy(other);
}

void utils::Vector2d::setPosition(
    const utils::Vector2d& v)
{
#ifdef BIORBD_USE_CASADI_MATH
    (*this)(0) = v(0);
    (*this)(1) = v(1);
#else
    *this = v;
#endif
}

void utils::Vector2d::setType()
{
    *m_typeOfNode = utils::VECTOR2D;
}

#ifdef BIORBD_USE_CASADI_MATH
utils::Scalar utils::Vector2d::x() const
{
    return (*this)[0];
}

utils::Scalar utils::Vector2d::y() const
{
    return (*this)[1];
}

void utils::Vector2d::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    *this = utils::Vector2d(other);
}

#endif
