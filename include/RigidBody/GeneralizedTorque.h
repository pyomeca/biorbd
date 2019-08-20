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
    GeneralizedTorque(const biorbd::utils::Vector& v);
    GeneralizedTorque(unsigned int i);
    GeneralizedTorque(const biorbd::rigidbody::Joints& j);

    biorbd::rigidbody::GeneralizedTorque timeDerivativeActivation(const biorbd::rigidbody::GeneralizedTorque &act);

};

}}

#endif // BIORBD_UTILS_GENERALIZED_TORQUE_H
