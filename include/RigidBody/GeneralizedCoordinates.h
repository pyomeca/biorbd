#ifndef BIORBD_UTILS_GENERALIZED_COORDINATES_H
#define BIORBD_UTILS_GENERALIZED_COORDINATES_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

class BIORBD_API GeneralizedCoordinates : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Create generalized coordinates
    /// 
    GeneralizedCoordinates();

    ///
    /// \brief Create generalized coordinates
    /// \param Q State vector of the internal joints
    ///
    GeneralizedCoordinates(const biorbd::rigidbody::GeneralizedCoordinates& Q);


    template<typename OtherDerived> GeneralizedCoordinates(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}

    ///
    /// \brief Create generalized coordinates of a vector at a specific position
    /// \param i Vector position
    ///
    GeneralizedCoordinates(unsigned int i);

    ///
    /// \brief Create generalized coordinates of specific joint
    /// \param j The joint
    ///
    GeneralizedCoordinates(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~GeneralizedCoordinates();

    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedCoordinates& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_GENERALIZED_COORDINATES_H
