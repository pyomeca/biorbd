#ifndef BIORBD_UTILS_GENERALIZED_VELOCITY_H
#define BIORBD_UTILS_GENERALIZED_VELOCITY_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

///
/// \brief Class GeneralizedVelocity
///
class BIORBD_API GeneralizedVelocity : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized velocity vector
    /// 
    GeneralizedVelocity();

    ///
    /// \brief Construct generalized velocity vector
    /// \param Q State vector of the internal joints
    ///
    GeneralizedVelocity(
            const biorbd::rigidbody::GeneralizedVelocity& Q);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct generalized velocity vector from another vector
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> GeneralizedVelocity(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedVelocity(
            const RigidBodyDynamics::Math::VectorNd& v);

#endif

    ///
    /// \brief Create generalized velocity vector of dimension nbQdot
    /// \param nbQdot number of degrees-of-freedom
    ///
    GeneralizedVelocity(
            unsigned int nbQdot);

    ///
    /// \brief Create generalized velocity vector from a joint Model
    /// \param j The joint model
    ///
    GeneralizedVelocity(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~GeneralizedVelocity();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized velocity vector
    ///
    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedVelocity& operator=(
                const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector::operator=(other);
            return *this;
        }
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief operator= For submatrices
    /// \param The vector to copy
    ///
    void operator=(
            const biorbd::utils::Vector& other);

    ///
    /// \brief operator= For submatrices
    /// \param The vector to copy
    ///
    void operator=(
            const MX_Xd_SubMatrix& other);

#endif
};

}}

#endif // BIORBD_UTILS_GENERALIZED_VELOCITY_H
