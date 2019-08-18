#define BIORBD_API_EXPORTS
#include "Utils/Vector.h"

#include "Utils/Error.h"
#include "Utils/String.h"

biorbd::utils::Vector::Vector()
{

}
biorbd::utils::Vector::Vector(const Eigen::VectorXd &v) :
    Eigen::VectorXd(v)
{

}
biorbd::utils::Vector::Vector(const biorbd::utils::Vector &v) :
    Eigen::VectorXd(v)
{

}
biorbd::utils::Vector::Vector(unsigned int i) :
    Eigen::VectorXd(i)
{

}
double biorbd::utils::Vector::norm(
        unsigned int p,
        bool skipRoot){
    biorbd::utils::Error::error(p >= 2, "p must be superior or equal to 2");

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

biorbd::utils::Vector biorbd::utils::Vector::normGradient(
        unsigned int p,
        bool skipRoot){
    biorbd::utils::Error::error(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        if (skipRoot)
            return biorbd::utils::Vector(*this * 2.);
        else
            return biorbd::utils::Vector(*this * 1.0/norm(2));
    } else {
        biorbd::utils::Vector res(static_cast<unsigned int>(size()));
        double normalized(std::pow(norm(), p-1));
        for (int i=0; i<size(); ++i)
            res[i] = (*this)[i] * std::pow(fabs((*this)[i]), p - 2);
        res /= normalized;
        if (skipRoot)
            biorbd::utils::Error::error(false, "skip root not implemented for p > 2");
        return res;
    }
}

biorbd::utils::Vector::~Vector()
{

}

Eigen::VectorXd biorbd::utils::Vector::vector() const
{
    return  this->block(0,0,this->rows(),1);
}
