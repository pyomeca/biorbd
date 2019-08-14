#ifndef BIORBD_UTILS_TAU_H
#define BIORBD_UTILS_TAU_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd {
namespace rigidbody {
class Joints;
}

namespace utils {

class BIORBD_API Tau : public biorbd::utils::Vector
{
public:
    Tau();
    Tau(const biorbd::utils::Vector& v);
    Tau(unsigned int i);
    Tau(const Eigen::VectorXd& v);
    Tau(const biorbd::rigidbody::Joints& j);

    biorbd::utils::Tau timeDerivativeActivation(const biorbd::utils::Tau &act);

};

}}

#endif // BIORBD_UTILS_TAU_H
