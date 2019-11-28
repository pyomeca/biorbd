#ifndef BIORBD_UTILS_GENERALIZED_TORQUE_H
#define BIORBD_UTILS_GENERALIZED_TORQUE_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

///
/// \brief Class GeneralizedTorque
///
class BIORBD_API GeneralizedTorque : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized torque
    ///
    GeneralizedTorque();

    ///
    /// \brief Construct generalized torque from anoter Generalized torque
    /// \param other The other generalized torque
    ///
    GeneralizedTorque(
            const biorbd::rigidbody::GeneralizedTorque& other);

    ///
    /// \brief Construct generalized torque
    /// \param other The other generalized torque
    ///
    template<typename OtherDerived> GeneralizedTorque(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}

    ///
    /// \brief Construct generalized torque of dimension nTorque
    /// \param nTorque Position of the vector
    ///
    GeneralizedTorque(
            unsigned int nTorque);

    ///
    /// \brief Construct generalized torque from a joint model
    /// \param j The joint model
    ///
    GeneralizedTorque(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized Torque
    ///
    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedTorque& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_GENERALIZED_TORQUE_H
