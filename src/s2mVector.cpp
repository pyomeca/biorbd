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
        res += std::pow(std::abs((*this)(i)), p);
    }

    return std::pow(res, 1.0/p);
}

s2mVector s2mVector::grad_norm(unsigned int p){
    s2mError::s2mAssert(p>=2, "p must be superior or equal to 2");
    s2mVector resvector(static_cast<unsigned int>(size()));
    double res(0);
    for(unsigned int i=0; i < size(); ++i){
        res += std::pow(std::abs((*this)(i)), p);
    }
    res = std::pow(res, 1.0/p-1.0);
    for(unsigned int i=0; i < size(); ++i){
        if ((*this)(i) < 0){
            resvector[i] = -res*std::pow(abs((*this)(i)), p-1);
        }
        else {
            resvector[i] = res*std::pow(abs((*this)(i)), p-1);
        }

    }
    return resvector;
}



s2mVector::~s2mVector() {}

Eigen::VectorXd s2mVector::vector() const{
    return  this->block(0,0,this->rows(),1);
}
