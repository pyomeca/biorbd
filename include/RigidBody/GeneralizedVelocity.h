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

    ///
    /// \brief Construct generalized velocity vector from another vector
    /// \param other Eigen matrix
    ///

    template<typename OtherDerived> GeneralizedVelocity(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}

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
};

}}

#endif // BIORBD_UTILS_GENERALIZED_VELOCITY_H
