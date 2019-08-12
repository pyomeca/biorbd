#ifndef S2M_GEN_COORD_H
#define S2M_GEN_COORD_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

class s2mJoints;
namespace biorbd { namespace utils {
class BIORBD_API GenCoord : public s2mVector
{
public:
    GenCoord();
    GenCoord(const GenCoord &Q);
    GenCoord(const s2mVector& v);
    GenCoord(const Eigen::VectorXd& v);
    GenCoord(unsigned int i);
    GenCoord(const s2mJoints& j);
    virtual ~GenCoord();

    GenCoord& operator=(const Eigen::VectorXd& vecX);

};

}}

#endif // S2M_GEN_COORD_H
