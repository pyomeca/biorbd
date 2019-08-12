#ifndef BIORBD_UTILS_TAU_H
#define BIORBD_UTILS_TAU_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

class s2mJoints;

namespace biorbd { namespace utils {

class BIORBD_API Tau : public biorbd::utils::Vector
{
public:
    Tau();
    Tau(const biorbd::utils::Vector& v);
    Tau(unsigned int i);
    Tau(const Eigen::VectorXd& v);
    Tau(const s2mJoints& j);

    biorbd::utils::Tau timeDerivativeActivation(const biorbd::utils::Tau &act);

};

}}

#endif // BIORBD_UTILS_TAU_H
