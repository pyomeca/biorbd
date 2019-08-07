#ifndef S2M_GEN_COORD_H
#define S2M_GEN_COORD_H

#include "biorbdConfig.h"
#include "s2mVector.h"

class s2mJoints;
class BIORBD_API s2mGenCoord : public s2mVector
{
public:
    s2mGenCoord();
    s2mGenCoord(const s2mGenCoord &Q);
    s2mGenCoord(const s2mVector& v);
    s2mGenCoord(const Eigen::VectorXd& v);
    s2mGenCoord(unsigned int i);
    s2mGenCoord(const s2mJoints& j);
    virtual ~s2mGenCoord();

    s2mGenCoord& operator=(const Eigen::VectorXd& vecX);

};

#endif // S2M_GEN_COORD_H
