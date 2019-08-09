#define BIORBD_API_EXPORTS
#include "s2mVector.h"


s2mVector::s2mVector()
{

}
s2mVector::s2mVector(const Eigen::VectorXd &v) :
    Eigen::VectorXd(v)
{

}
s2mVector::s2mVector(const s2mVector &v) :
    Eigen::VectorXd(v)
{

}
s2mVector::s2mVector(unsigned int i) :
    Eigen::VectorXd(i)
{

}

s2mVector::~s2mVector()
{

}

Eigen::VectorXd s2mVector::vector() const
{
    return  this->block(0,0,this->rows(),1);
}
