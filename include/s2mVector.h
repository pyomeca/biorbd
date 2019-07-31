#ifndef S2M_VECTOR_H
#define S2M_VECTOR_H
    #include <Eigen/Dense>
	#include "biorbdConfig.h"
    #include <math.h>
    #include "s2mError.h"

class BIORBD_API s2mVector : public Eigen::VectorXd
{
public:
    s2mVector();
    s2mVector(const Eigen::VectorXd& v);
    s2mVector(const s2mVector& v);
    s2mVector(unsigned int i);
    ~s2mVector();
    virtual Eigen::VectorXd vector() const;
    double norm(unsigned int p = 2, bool skipRoot = false);
    s2mVector norm_gradient(unsigned int p = 2, bool skipRoot = false);

protected:
private:

};

#endif // S2M_VECTOR_H
