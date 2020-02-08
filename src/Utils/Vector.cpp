#define BIORBD_API_EXPORTS
#include "Utils/Vector.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

biorbd::utils::Vector::Vector() :
    RigidBodyDynamics::Math::VectorNd()
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::utils::Vector::Vector(
        const biorbd::utils::Vector& v) :
    RigidBodyDynamics::Math::VectorNd (v)
{

}

biorbd::utils::Vector::Vector(
        const MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::VectorNd (m)
{

}

biorbd::utils::Vector::Vector(
        const biorbd::utils::Vector3d& v) :
    RigidBodyDynamics::Math::VectorNd (v)
{

}


#endif

biorbd::utils::Vector::Vector(
        unsigned int size) :
    RigidBodyDynamics::Math::VectorNd(size)
{

}

RigidBodyDynamics::Math::Scalar biorbd::utils::Vector::norm(
        unsigned int p,
        bool skipRoot) const 
{
    biorbd::utils::Error::check(p >= 2, "p must be superior or equal to 2");

    if (p == 2){
        RigidBodyDynamics::Math::Scalar n = dot(*this);
        if (skipRoot)
            return n;
        else
            return std::sqrt(n);
    } else {
        RigidBodyDynamics::Math::Scalar res(0);
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
