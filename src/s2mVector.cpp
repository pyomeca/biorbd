#define BIORBD_API_EXPORTS
#include "../include/s2mVector.h"


s2mVector::s2mVector() {}

s2mVector::s2mVector(const Eigen::VectorXd &v) : Eigen::VectorXd(v) {}

s2mVector::s2mVector(const s2mVector &v) : Eigen::VectorXd(v) {}

s2mVector::s2mVector(unsigned int i) : Eigen::VectorXd(i) {}

double s2mVector::norm(unsigned int p){
    s2mError::s2mAssert(p>=2, "p must be superior or equal to 2");

    double res(0);
    for(unsigned int i=0; i < size(); ++i){
        res += std::pow(fabs((*this)(i)), p);
    }

    return std::pow(res, 1.0/p);
}

double s2mVector::normdot(unsigned int p){
    s2mError::s2mAssert(p>=2, "p must be superior or equal to 2");

    double res1(0);
    double res2(0);
    for(unsigned int i=0; i < size(); ++i){
        res1 += std::pow(p*fabs((*this)(i)), p-1);
        res2 += std::pow(fabs((*this)(i)), p);
    }
    return res1*std::pow(res2, 1.0/p-1.0);
}



s2mVector::~s2mVector() {}

Eigen::VectorXd s2mVector::vector() const{
    return  this->block(0,0,this->rows(),1);
}
