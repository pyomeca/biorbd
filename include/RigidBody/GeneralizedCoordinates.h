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
    GeneralizedCoordinates(const biorbd::rigidbody::GeneralizedCoordinates &Q);
    GeneralizedCoordinates(const biorbd::utils::Vector& v);
    GeneralizedCoordinates(unsigned int i);
    GeneralizedCoordinates(const biorbd::rigidbody::Joints& j);
    virtual ~GeneralizedCoordinates();

    biorbd::rigidbody::GeneralizedCoordinates& operator=(const biorbd::utils::Vector& vecX);
    biorbd::rigidbody::GeneralizedCoordinates& operator=(const Eigen::VectorXd& vecX);

};

}}

#endif // BIORBD_UTILS_GENERALIZED_COORDINATES_H
