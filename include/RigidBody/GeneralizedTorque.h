#ifndef BIORBD_UTILS_GENERALIZED_TORQUE_H
#define BIORBD_UTILS_GENERALIZED_TORQUE_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;

class BIORBD_API GeneralizedTorque : public biorbd::utils::Vector
{
public:
    GeneralizedTorque();
    GeneralizedTorque(const biorbd::rigidbody::GeneralizedTorque& Q);
    template<typename OtherDerived> GeneralizedTorque(const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other){}
    GeneralizedTorque(unsigned int i);
    GeneralizedTorque(const biorbd::rigidbody::Joints& j);

    biorbd::rigidbody::GeneralizedTorque timeDerivativeActivation(const biorbd::rigidbody::GeneralizedTorque &act);

    template<typename OtherDerived>
        biorbd::rigidbody::GeneralizedTorque& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->biorbd::utils::Vector::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_UTILS_GENERALIZED_TORQUE_H
