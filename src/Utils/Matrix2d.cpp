#define BIORBD_API_EXPORTS
#include "Utils/Matrix2d.h"

#include "Utils/Vector.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

utils::Matrix2d::Matrix2d() :
    RigidBodyDynamics::Math::Matrix2d()
{

}

#ifdef BIORBD_USE_CASADI_MATH

utils::Matrix2d::Matrix2d(
    const RBDLCasadiMath::MX_Xd_static<2, 2>& v) :
    RigidBodyDynamics::Math::Matrix2d (v)
{

}

utils::Matrix2d::Matrix2d(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::Matrix2d (m)
{

}

void utils::Matrix2d::operator=(
    const RBDLCasadiMath::MX_Xd_static<2, 2> &other)
{
    this->MX_Xd_static<2, 2>::operator=(other);
}

void utils::Matrix2d::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_static<2, 2>::operator=(other);
}

#endif
