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
    GeneralizedCoordinates();
    template<typename OtherDerived> GeneralizedCoordinates(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}
    GeneralizedCoordinates(unsigned int i);
    GeneralizedCoordinates(const biorbd::rigidbody::Joints& j);

    virtual ~GeneralizedCoordinates();

    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedCoordinates& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_GENERALIZED_COORDINATES_H
