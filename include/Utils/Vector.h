#ifndef BIORBD_UTILS_VECTOR_H
#define BIORBD_UTILS_VECTOR_H

#include "biorbdConfig.h"
#include "rbdl_math.h"
#include "Utils/Scalar.h"

namespace biorbd {
namespace utils {
class Vector3d;

///
/// \brief Wrapper of the Eigen VectorXd
///
#ifdef SWIG
class BIORBD_API Vector
#else
class BIORBD_API Vector : public RigidBodyDynamics::Math::VectorNd
#endif
{
public:
    ///
    /// \brief Construct vector
    ///
    Vector();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct vector from Eigen matrix
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> Vector(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::VectorXd(other){}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param other The vector to copy
    ///
    Vector(
            const biorbd::utils::Vector& v);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param other The vector to copy
    ///
    Vector(
            const RigidBodyDynamics::Math::VectorNd& v);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    Vector(
            const casadi::MX& v);

    ///
    /// \brief Construct vector from Casadi matrix
    /// \param other The vector to copy
    ///
    Vector(
            const MX_Xd_SubMatrix& m);

    ///
    /// \brief Construct vector from Casadi matrix
    /// \param v The vector to copy
    ///
    Vector(
            const biorbd::utils::Vector3d& v);

#endif

    ///
    /// \brief Construct vector of dimension size
    /// \param size The length of the vector
    ///
    Vector(
            unsigned int size);

    ///
    /// \brief Return the Euclidian p-norm of the vector
    /// \param p the factor of the p-norm
    /// \param skipRoot To perform or not the sqrt_p()
    /// \return The norm of the vector
    ///
    biorbd::utils::Scalar norm(
            unsigned int p = 2,
            bool skipRoot = false) const;

    ///
    /// \brief Return the gradient of the p-norm
    /// \param p the factor of the p-norm
    /// \param skipRoot To perform or not the sqrt_p()
    /// \return The gradient of the norm
    ///
    biorbd::utils::Vector normGradient(
            unsigned int p = 2,
            bool skipRoot = false);

#ifdef BIORBD_USE_EIGEN3_MATH
    /// 
    /// \brief Allow the use operator= on vector
    /// \param other The other matrix
    /// 
    template<typename OtherDerived>
        biorbd::utils::Vector& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::VectorXd::operator=(other);
            return *this;
        }
#endif
#ifdef BIORBD_USE_CASADI_MATH

#ifndef SWIG
    ///
    /// \brief operator= For submatrices
    /// \param other The vector to copy
    ///
    void operator=(
            const biorbd::utils::Vector& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The vector to copy
    ///
    void operator=(
            const MX_Xd_SubMatrix& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The vector to copy
    ///
    void operator=(
            const casadi::MX& other);
#endif

#endif
};

}}

#endif // BIORBD_UTILS_VECTOR_H
