#define BIORBD_API_EXPORTS
#include "Utils/SpatialTransform.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "Utils/Matrix3d.h"
#include "Utils/RotoTrans.h"

using namespace BIORBD_NAMESPACE;

utils::SpatialTransform::SpatialTransform() :
    RigidBodyDynamics::Math::SpatialTransform()
{

}

utils::SpatialTransform::SpatialTransform(
    const utils::SpatialTransform& other) :
    RigidBodyDynamics::Math::SpatialTransform(other)
{

}

utils::SpatialTransform::SpatialTransform(
    const utils::Matrix3d& rotation,
    const utils::Vector3d& translation) :
    RigidBodyDynamics::Math::SpatialTransform(rotation, translation)
{

}

utils::SpatialTransform::SpatialTransform(
    const utils::RotoTrans& rototrans) :
    RigidBodyDynamics::Math::SpatialTransform(rototrans.block<3, 3>(0, 0), rototrans.trans())
{

}

utils::SpatialTransform::SpatialTransform(
    const RigidBodyDynamics::Math::SpatialTransform& other) :
    RigidBodyDynamics::Math::SpatialTransform(other)
{

}

void utils::SpatialTransform::operator=(
    const utils::SpatialTransform&other)
{
    this->RigidBodyDynamics::Math::SpatialTransform::operator=(other);
}

utils::RotoTrans utils::SpatialTransform::rototrans() const {
    return RotoTrans(*this);
}


utils::Matrix3d utils::SpatialTransform::rotation() const {
    return utils::Matrix3d(this->E);
}

utils::Vector3d utils::SpatialTransform::translation() const {
    return this->r;
}
