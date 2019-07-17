#define BIORBD_API_EXPORTS
#include "../include/s2mVector.h"


s2mVector::s2mVector() {}

s2mVector::s2mVector(const Eigen::VectorXd &v) : Eigen::VectorXd(v) {}

s2mVector::s2mVector(const s2mVector &v) : Eigen::VectorXd(v) {}

s2mVector::s2mVector(unsigned int i) : Eigen::VectorXd(i) {}

double s2mVector::norm(unsigned int p, bool skipRoot){
    s2mError::s2mAssert(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        double n = this->transpose() * *this;
        if (skipRoot)
            return n;
        else
            return std::sqrt(n);
    } else {
        double res(0);
        for(unsigned int i=0; i < size(); ++i)
            res += std::pow(fabs((*this)[i]), p);
        if (skipRoot)
            return res;
        else
            return std::pow(res, 1.0/p);
    }
}

s2mVector s2mVector::norm_gradient(unsigned int p, bool skipRoot){
    s2mError::s2mAssert(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        if (skipRoot)
            return s2mVector(*this * 2.);
        else
            return s2mVector(*this * 1.0/norm(2));
    } else {
        s2mVector res(static_cast<unsigned int>(size()));
        double normalized(std::pow(norm(), p-1));
        for (int i=0; i<size(); ++i)
            res[i] = (*this)[i] * std::pow(fabs((*this)[i]), p - 2);
        res /= normalized;
        if (skipRoot)
            s2mError::s2mAssert(false, "skip root not implemented for p > 2");
        return res;
    }
}



s2mVector::~s2mVector() {}

Eigen::VectorXd s2mVector::vector() const{
    return  this->block(0,0,this->rows(),1);
}
