#ifndef BIORBD_UTILS_GENERALIZED_ACCELERATION_H
#define BIORBD_UTILS_GENERALIZED_ACCELERATION_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

///
/// \brief Class GeneralizedAcceleration
///
class BIORBD_API GeneralizedAcceleration : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized acceleration vector
    /// 
    GeneralizedAcceleration();

    ///
    /// \brief Construct generalized acceleration vector
    /// \param Q State vector of the internal joints
    ///
    GeneralizedAcceleration(
            const biorbd::rigidbody::GeneralizedAcceleration& Q);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct generalized acceleration vector from another vector
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> GeneralizedAcceleration(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedAcceleration(
            const RigidBodyDynamics::Math::VectorNd& v);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedAcceleration(
            const casadi::MX& v);

#endif

    ///
    /// \brief Create generalized acceleration vector of dimension nQddot
    /// \param nQddot number of degrees-of-freedom
    ///
    GeneralizedAcceleration(
            unsigned int nQddot);

    ///
    /// \brief Create generalized acceleration vector from a joint Model
    /// \param j The joint model
    ///
    GeneralizedAcceleration(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~GeneralizedAcceleration();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized Coordinate
    ///
    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedAcceleration& operator=(
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

    ///
    /// \brief operator= For casadi
    /// \param The vector to copy
    ///
    void operator=(
            const casadi::MX& other);

#endif
};

}}

#endif // BIORBD_UTILS_GENERALIZED_ACCELERATION_H
