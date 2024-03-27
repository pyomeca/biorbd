#ifndef BIORBD_UTILS_MATRIX3D_H
#define BIORBD_UTILS_MATRIX3D_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class String;

///
/// \brief A wrapper for the Eigen::MatrixXd
///
#ifdef SWIG
class BIORBD_API Matrix3d
#else
class BIORBD_API Matrix3d : public RigidBodyDynamics::Math::Matrix3d
#endif
{
public:
    ///
    /// \brief Construct matrix
    ///
    Matrix3d();

    /// 
    /// \brief Construct a new Matrix3d from other
    /// \param other The other matrix
    /// 
    Matrix3d(const Matrix3d& other);

    ///
    /// \brief Rotation Construct a generic 3d matrix by elements
    /// \param v00 Row 0, Col 0
    /// \param v01 Row 0, Col 1
    /// \param v02 Row 0, Col 2
    /// \param v10 Row 1, Col 0
    /// \param v11 Row 1, Col 1
    /// \param v12 Row 1, Col 2
    /// \param v20 Row 2, Col 0
    /// \param v21 Row 2, Col 1
    /// \param v22 Row 2, Col 2
    ///
    Matrix3d(const Scalar& v00, const Scalar& v01, const Scalar& v02,
             const Scalar& v10, const Scalar& v11, const Scalar& v12,
             const Scalar& v20, const Scalar& v21, const Scalar& v22);


#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct matrix from another Eigen matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived> Matrix3d(const Eigen::MatrixBase<OtherDerived>&
                                           other) :
        RigidBodyDynamics::Math::Matrix3d(other) {}
#endif

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Return the 1-norm of the matrix
    /// \return The 1-norm of the matrix
    ///
    utils::Scalar normOne() const;

    ///
    /// \brief Return the inf-norm of the matrix
    /// \return The inf-norm of the matrix
    ///
    utils::Scalar normInf() const;

    ///
    /// \brief Produce an orthonormal matrix from the current matrix
    /// \return The othonormal matrix
    ///
    /// Code adapted from https://github.com/brainexcerpts/3x3_polar_decomposition/blob/master/src/polar_decomposition_3x3.inl
    ///
    utils::Matrix3d orthoNormalize() const;
#endif

    /// 
    /// \brief Creates a base 3d matrix from a euler sequence of exactly 3 elements
    /// \return The base matrix
    /// 
    static utils::Matrix3d fromEulerSequence(const utils::String& seq);

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix3d(
        const RBDLCasadiMath::MX_Xd_static<3, 3>& other);

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix3d(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

#endif

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= with matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived>
    Matrix3d& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Matrix3d::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_static<3, 3>& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif

#endif
};

}
}

#endif // BIORBD_UTILS_MATRIX3D_H
