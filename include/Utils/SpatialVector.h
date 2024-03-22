#ifndef BIORBD_UTILS_SPATIAL_VECTOR_H
#define BIORBD_UTILS_SPATIAL_VECTOR_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
    class Vector3d;

///
/// \brief Wrapper of the Eigen::Matrix<double, 6, 1> or Casadi::MX(6, 1)
///
#ifdef SWIG
class BIORBD_API SpatialVector
#else
class BIORBD_API SpatialVector : public RigidBodyDynamics::Math::SpatialVector
#endif
{
public:
    ///
    /// \brief Construct SpatialVector
    ///
    SpatialVector();

    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param other The SpatialVector to copy
    ///
    SpatialVector(
        const SpatialVector& other);

    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param moment The vector for the moment
    /// \param forceThe vector for the force
    ///
    SpatialVector(
        const Vector3d& moment, 
        const Vector3d& force);

#ifndef SWIG
    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param other The SpatialVector to copy
    ///
    SpatialVector(
        const RigidBodyDynamics::Math::SpatialVector& other);
#endif

    ///
    /// \brief Construct SpatialVector by its values
    /// \param mx Moment about the X axis
    /// \param my Moment about the Y axis
    /// \param mz Moment about the Z axis
    /// \param fx Froce along the X axis
    /// \param fy Force along the Y axis
    /// \param fz Force along the Z axis
    ///
    SpatialVector(
        Scalar mx, Scalar my, Scalar mz,
        Scalar fx, Scalar fy, Scalar fz
    );

#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param v The SpatialVector to copy
    ///
    SpatialVector(
        const casadi::MX& v);

    ///
    /// \brief Construct SpatialVector from Casadi matrix
    /// \param other The SpatialVector to copy
    ///
    SpatialVector(
        const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

#ifndef SWIG
    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const SpatialVector& other);
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use operator= on SpatialVector
    /// \param other The other matrix
    ///
    template<typename OtherDerived>
    SpatialVector& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Matrix<double, 6, 1>::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const casadi::MX& other);
#endif

    /// 
    /// \brief Return the force part of the vector (last three elements)
    /// 
    utils::Vector3d force() const;

    /// 
    /// \brief Return the moment part of the vector (first three elements)
    /// 
    utils::Vector3d moment() const;

#endif
};

}
}

#endif // BIORBD_UTILS_SPATIAL_VECTOR_H
