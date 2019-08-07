#ifndef S2MTAU_H
#define S2MTAU_H

#include "biorbdConfig.h"
#include "s2mVector.h"

class s2mJoints;
class BIORBD_API s2mTau : public s2mVector
{
public:
    s2mTau() {}
    s2mTau(const s2mVector& v) : s2mVector(v) {}
    s2mTau(unsigned int i) : s2mVector(i) {}
    s2mTau(const Eigen::VectorXd& v) : s2mVector(v) {}
    s2mTau(const s2mJoints& j);

    s2mTau timeDerivativeActivation(const s2mTau &act);

};

#endif // S2MTAU_H
