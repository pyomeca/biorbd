#ifndef BIORBD_UTILS_GENERALIZED_COORDINATES_H
#define BIORBD_UTILS_GENERALIZED_COORDINATES_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

///
/// \brief Class GeneralizedCoordinates 
///
class BIORBD_API GeneralizedCoordinates : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized coordinates
    /// 
    GeneralizedCoordinates();

    ///
    /// \brief Construct generalized coordinates
    /// \param Q State vector of the internal joints
    ///
    GeneralizedCoordinates(
            const biorbd::rigidbody::GeneralizedCoordinates& Q);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct generalized coordinates from another vector
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> GeneralizedCoordinates(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedCoordinates(
            const RigidBodyDynamics::Math::VectorNd& v);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedCoordinates(
            const casadi::MX& v);

#endif

    ///
    /// \brief Create generalized coordinates of dimension nbQ
    /// \param nbQ number of degrees-of-freedom
    ///
    GeneralizedCoordinates(
            unsigned int nbQ);

    ///
    /// \brief Create generalized coordinates from a joint Model
    /// \param j The joint model
    ///
    GeneralizedCoordinates(
            const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~GeneralizedCoordinates();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized Coordinate
    ///
    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedCoordinates& operator=(
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

#endif // BIORBD_UTILS_GENERALIZED_COORDINATES_H
