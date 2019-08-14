#ifndef BIORBD_UTILS_GEN_COORD_H
#define BIORBD_UTILS_GEN_COORD_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd { namespace rigidbody {
class Joints;
}}

namespace biorbd { namespace utils {

class BIORBD_API GenCoord : public Vector
{
public:
    GenCoord();
    GenCoord(const biorbd::utils::GenCoord &Q);
    GenCoord(const biorbd::utils::Vector& v);
    GenCoord(const Eigen::VectorXd& v);
    GenCoord(unsigned int i);
    GenCoord(const biorbd::rigidbody::Joints& j);
    virtual ~GenCoord();

    biorbd::utils::GenCoord& operator=(const Eigen::VectorXd& vecX);

};

}}

#endif // BIORBD_UTILS_GEN_COORD_H
