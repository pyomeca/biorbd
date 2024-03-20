#ifndef BIORBD_UTILS_VECTOR2D_H
#define BIORBD_UTILS_VECTOR2D_H

#include <memory>
#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Node.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class RotoTrans;
class String;
class Vector;

///
/// \brief Wrapper around Eigen Vector2d and attach it to a parent
///
#ifdef SWIG
class BIORBD_API Vector2d
#else
class BIORBD_API Vector2d : public RigidBodyDynamics::Math::Vector2d,
    public Node
#endif
{
public:
    ///
    /// \brief Construct 2D vector
    ///
    Vector2d();

    ///
    /// \brief Construct 2D vector
    /// \param x X-Component of the vector
    /// \param y Y-Component of the vector
    ///
    Vector2d(
        const Scalar& x,
        const Scalar& y);

    ///
    /// \brief Construct a 2D vector
    /// \param x X-Component of the vector
    /// \param y Y-Component of the vector
    /// \param name Name of the vector
    /// \param parentName Name of the parent segment
    ///
    Vector2d(
        const Scalar& x,
        const Scalar& y,
        const String &name,
        const String &parentName);

    ///
    /// \brief Construct a 2D vector
    /// \param vec The vector to copy
    /// \param name Name of the vector
    /// \param parentName Name of the parent segment
    ///
    Vector2d(
        const Vector2d vec,
        const String &name,
        const String &parentName);

    ///
    /// \brief Construct a 2D vector from a Casadi 2D vector (drop the trailling 1)
    /// \param other The Casadi 2D vector
    ///
    Vector2d(
        const RigidBodyDynamics::Math::Vector2d& other);

    ///
    /// \brief Construct a 2D vector from a Casadi ND vector (drop the trailling 1)
    /// \param other The Casadi ND vector
    ///
    Vector2d(
        const RigidBodyDynamics::Math::VectorNd& other);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a 2D vector
    /// \param other The other vector
    ///
    template<typename OtherDerived> Vector2d(
        const Eigen::MatrixBase<OtherDerived>& other) :
        RigidBodyDynamics::Math::Vector2d(other), Node ()
    {
    }

    ///
    /// \brief Construct a 2D vector
    /// \param other Position of the vector (eigen matrix)
    /// \param name Name of the vector
    /// \param parentName The name of the parent segment
    ///
    template<typename OtherDerived> Vector2d(
        const Eigen::MatrixBase<OtherDerived>& other,
        const String &name,
        const String &parentName) :
        RigidBodyDynamics::Math::Vector2d(other), Node (name, parentName)
    {

    }
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct a 2D vector from a Casadi ND vector (drop the trailling 1)
    /// \param other The Casadi ND vector
    ///
    Vector2d(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif

    ///
    /// \brief Deep copy of a 2D vector
    /// \return A deep copy of a 2D vector
    ///
    Vector2d DeepCopy() const;

    ///
    /// \brief Deep copy of a 2D vector into another 2D vector
    /// \param other The 2D vector to copy
    ///
    void DeepCopy(const Vector2d& other);

    ///
    /// \brief Set a new position
    /// \param v The new position
    ///
    void setPosition(
        const Vector2d& v);

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= on 2D vector with any eigen vector
    /// \param other The eigen matrix
    ///
    template<typename OtherDerived>
    Vector2d& operator=(const Eigen::MatrixBase <OtherDerived>&other)
    {
        this->Eigen::Vector2d::operator=(other);
        return *this;
    }
#endif

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Accessor for the first element
    /// \return The first element
    ///
    Scalar x() const;

    ///
    /// \brief Accessor for the second element
    /// \return The second element
    ///
    Scalar y() const;

    ///
    /// \brief Construct a 2D vector from a Casadi 4D vector (drop the trailling 1)
    /// \param other The Casadi 4D vector
    ///
    template<unsigned int i, unsigned int j>
    void operator=(
        const MX_Xd_static<i, j>& other)
    {
        this->block<2, 1>(0, 0) = other;
    }

    ///
    /// \brief operator= To copy a submatrix
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

#endif

#endif

protected:
    ///
    /// \brief Set the type Vector2d
    ///
    void setType();
};

}
}

#endif // BIORBD_UTILS_VECTOR2D_H
