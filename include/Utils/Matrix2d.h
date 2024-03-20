#ifndef BIORBD_UTILS_MATRIX2D_H
#define BIORBD_UTILS_MATRIX2D_H

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
class BIORBD_API Matrix2d
#else
class BIORBD_API Matrix2d : public RigidBodyDynamics::Math::Matrix2d
#endif
{
public:
    ///
    /// \brief Construct matrix
    ///
    Matrix2d();

    ///
    /// \brief Rotation Construct a generic 2d matrix by elements
    /// \param v00 Row 0, Col 0
    /// \param v01 Row 0, Col 1
    /// \param v10 Row 1, Col 0
    /// \param v11 Row 1, Col 1
    ///
    Matrix2d(const Scalar& v00, const Scalar& v01,
             const Scalar& v10, const Scalar& v11);


#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct matrix from another Eigen matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived> Matrix2d(const Eigen::MatrixBase<OtherDerived>&
                                           other) :
        RigidBodyDynamics::Math::Matrix2d(other) {}
#endif

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix2d(
        const RBDLCasadiMath::MX_Xd_static<2, 2>& other);

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix2d(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

#endif

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= with matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived>
    Matrix2d& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Matrix2d::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_static<2, 2>& other);

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

#endif // BIORBD_UTILS_MATRIX2D_H
