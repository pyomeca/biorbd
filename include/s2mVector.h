#ifndef S2MVECTOR_H
#define S2MVECTOR_H
    #include <Eigen/Dense>
	#include "biorbdConfig.h"

class BIORBD_API s2mVector : public Eigen::VectorXd
{
public:
    s2mVector();
    s2mVector(const Eigen::VectorXd& v);
    s2mVector(const s2mVector& v);
    s2mVector(unsigned int i);
    virtual ~s2mVector();
    Eigen::VectorXd vector() const;

protected:
private:

};

#endif // S2MVECTOR_H
