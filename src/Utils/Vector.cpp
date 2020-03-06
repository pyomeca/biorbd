#define BIORBD_API_EXPORTS
#include "Utils/Vector.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

biorbd::utils::Vector::Vector() :
    RigidBodyDynamics::Math::VectorNd()
{

}

biorbd::utils::Vector::Vector(
        unsigned int size) :
    RigidBodyDynamics::Math::VectorNd(size)
{

}

biorbd::utils::Vector::Vector(
        const biorbd::utils::Vector& v) :
    RigidBodyDynamics::Math::VectorNd (v)
{

}

biorbd::utils::Vector::Vector(
        const RigidBodyDynamics::Math::VectorNd &v) :
    RigidBodyDynamics::Math::VectorNd (v)
{

}

biorbd::utils::Vector::Vector(
        const biorbd::utils::Vector3d& v) :
    RigidBodyDynamics::Math::VectorNd (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::utils::Vector::Vector(const casadi::MX &v) :
    RigidBodyDynamics::Math::VectorNd(v)
{

}

biorbd::utils::Vector::Vector(
        const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::VectorNd (m)
{

}

#endif

biorbd::utils::Scalar biorbd::utils::Vector::norm(
        unsigned int p,
        bool skipRoot) const 
{
    biorbd::utils::Error::check(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        biorbd::utils::Scalar n = dot(*this);
        if (skipRoot)
            return n;
        else
            return std::sqrt(n);
    } else {
        biorbd::utils::Scalar res(0);
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
        bool skipRoot)
{
    biorbd::utils::Error::check(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        if (skipRoot)
            return biorbd::utils::Vector(*this * 2.);
        else
            return biorbd::utils::Vector(*this * 1.0/norm(2));
    } else {
        biorbd::utils::Vector res(static_cast<unsigned int>(size()));
        double normalized(std::pow(norm(), p-1));
        for (unsigned int i=0; i<size(); ++i)
            res[i] = (*this)[i] * std::pow(fabs((*this)[i]), p - 2);
        res /= normalized;
        if (skipRoot)
            biorbd::utils::Error::raise("skip root not implemented for p > 2");
        return res;
    }
}

void biorbd::utils::Vector::operator=(
        const biorbd::utils::Vector &other)
{
    this->RigidBodyDynamics::Math::VectorNd::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::utils::Vector::operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_dynamic::operator=(other);
}

void biorbd::utils::Vector::operator=(
        const casadi::MX &other)
{
    this->MX_Xd_dynamic::operator=(other);
}

#endif
