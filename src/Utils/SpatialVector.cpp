#define BIORBD_API_EXPORTS
#include "Utils/SpatialVector.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

utils::SpatialVector::SpatialVector() :
    RigidBodyDynamics::Math::SpatialVector()
{

}

utils::SpatialVector::SpatialVector(
    const utils::SpatialVector& other) :
    RigidBodyDynamics::Math::SpatialVector (other)
{

}

utils::SpatialVector::SpatialVector(
    Scalar v1, Scalar v2, Scalar v3,
    Scalar v4, Scalar v5, Scalar v6) :
    RigidBodyDynamics::Math::SpatialVector (v1, v2, v3, v4, v5, v6)
{

}

#ifdef BIORBD_USE_CASADI_MATH

utils::SpatialVector::SpatialVector(
    const casadi::MX &v) :
    RigidBodyDynamics::Math::SpatialVector(v)
{

}

utils::SpatialVector::SpatialVector(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::SpatialVector (m)
{

}

#endif

void utils::SpatialVector::operator=(
    const utils::SpatialVector &other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void utils::SpatialVector::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

void utils::SpatialVector::operator=(
    const casadi::MX &other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

#endif
