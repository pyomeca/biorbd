#define BIORBD_API_EXPORTS
#include "Utils/Matrix3d.h"

#include "Utils/Vector.h"

using namespace BIORBD_NAMESPACE;

utils::Matrix3d::Matrix3d() :
    RigidBodyDynamics::Math::Matrix3d()
{

}

utils::Matrix3d::Matrix3d(
    const utils::Scalar &v00, const utils::Scalar &v01, const utils::Scalar &v02,
    const utils::Scalar &v10, const utils::Scalar &v11, const utils::Scalar &v12,
    const utils::Scalar &v20, const utils::Scalar &v21, const utils::Scalar &v22) :
    RigidBodyDynamics::Math::Matrix3d(v00, v01, v02, v10, v11, v12, v20, v21, v22)
{

}


#ifdef BIORBD_USE_CASADI_MATH

utils::Matrix3d::Matrix3d(
    const RBDLCasadiMath::MX_Xd_static<3, 3>& v) :
    RigidBodyDynamics::Math::Matrix3d (v)
{

}

utils::Matrix3d::Matrix3d(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::Matrix3d (m)
{

}

void utils::Matrix3d::operator=(
    const RBDLCasadiMath::MX_Xd_static<3, 3> &other)
{
    this->MX_Xd_static<3, 3>::operator=(other);
}

void utils::Matrix3d::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_static<3, 3>::operator=(other);
}

#endif
