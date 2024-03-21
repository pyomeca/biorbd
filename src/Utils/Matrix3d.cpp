#define BIORBD_API_EXPORTS
#include "Utils/Matrix3d.h"

#include "Utils/Vector.h"
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

utils::Matrix3d::Matrix3d() :
    RigidBodyDynamics::Math::Matrix3d()
{

}

utils::Matrix3d::Matrix3d(
    const utils::Matrix3d& other) :
    RigidBodyDynamics::Math::Matrix3d(other)
{

}

utils::Matrix3d::Matrix3d(
    const utils::Scalar &v00, const utils::Scalar &v01, const utils::Scalar &v02,
    const utils::Scalar &v10, const utils::Scalar &v11, const utils::Scalar &v12,
    const utils::Scalar &v20, const utils::Scalar &v21, const utils::Scalar &v22) :
    RigidBodyDynamics::Math::Matrix3d(v00, v01, v02, v10, v11, v12, v20, v21, v22)
{

}

#ifndef BIORBD_USE_CASADI_MATH
utils::Scalar utils::Matrix3d::normOne() const
{
    utils::Scalar value = 0.0;
    for (size_t i = 0; i < 3; ++i)
    {
        utils::Scalar col_sum = fabs((*this)(0, i)) + fabs((*this)(1, i)) + fabs((*this)(2, i));
        if (col_sum > value)
            value = col_sum;
    }
    return value;
}

utils::Scalar utils::Matrix3d::normInf() const
{
    utils::Scalar value = 0.0;
    for (size_t i = 0; i < 3; ++i)
    {
        utils::Scalar row_sum = fabs((*this)(i, 0)) + fabs((*this)(i, 1)) + fabs((*this)(i, 2));
        if (row_sum > value)
            value = row_sum;
    }
    return value;
}

utils::Matrix3d utils::Matrix3d::orthoNormalize() const
{
#ifdef BIORBD_USE_EIGEN3_MATH
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(*this, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
#else
#error "SVD decomposition not implemented for non-eigen3 backend"
#endif
}
#endif

utils::Matrix3d utils::Matrix3d::fromEulerSequence(
    const utils::String& seq)
{
    utils::Error::check(seq.length() == 3, "The angle sequence should be exactly 3.");

    utils::Matrix3d baseUnitMatrix(utils::Matrix3d::Zero());
    for (int i = 0; i < seq.length(); i++) {
        int indexSeq;
        if (seq[i] == 'x') {
            indexSeq = 0;
        }
        else if (seq[i] == 'y') {
            indexSeq = 1;
        }
        else if (seq[i] == 'z') {
            indexSeq = 2;
        }
        else {
            utils::Error::raise("Angle sequence must be composed of x, y, and/or z.");
        }
        baseUnitMatrix(i, indexSeq) = 1;
    }
    return baseUnitMatrix;
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
