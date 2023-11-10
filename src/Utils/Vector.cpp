#define BIORBD_API_EXPORTS
#include "Utils/Vector.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

utils::Vector::Vector() :
    RigidBodyDynamics::Math::VectorNd()
{

}

utils::Vector::Vector(
    size_t size) :
    RigidBodyDynamics::Math::VectorNd(static_cast<unsigned int>(size))
{

}

utils::Vector::Vector(
    const utils::Vector& other) :
    RigidBodyDynamics::Math::VectorNd (other)
{

}

utils::Vector::Vector(
    const RigidBodyDynamics::Math::VectorNd &other) :
    RigidBodyDynamics::Math::VectorNd (other)
{

}

utils::Vector::Vector(
    const utils::Vector3d& other) :
    RigidBodyDynamics::Math::VectorNd (other)
{

}

#ifdef BIORBD_USE_CASADI_MATH

utils::Vector::Vector(const casadi::MX &other) :
    RigidBodyDynamics::Math::VectorNd(other)
{

}

utils::Vector::Vector(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other) :
    RigidBodyDynamics::Math::VectorNd (other)
{

}

#endif

utils::Scalar utils::Vector::norm(
    size_t p,
    bool skipRoot) const
{
    utils::Error::check(p >= 2, "p must be superior or equal to 2");

    if (p == 2) {
        utils::Scalar n = dot(*this);
        if (skipRoot) {
            return n;
        } else {
            return std::sqrt(n);
        }
    } else {
        utils::Scalar res(0);
        for(unsigned int i=0; i < static_cast<unsigned int>(size()); ++i) {
            res += std::pow(fabs((*this)[i]), static_cast<unsigned int>(p));
        }
        if (skipRoot) {
            return res;
        } else {
            return std::pow(res, 1.0/p);
        }
    }
}

utils::Vector utils::Vector::normGradient(
    size_t p,
    bool skipRoot)
{
    utils::Error::check(p >= 2, "p must be superior or equal to 2");

    if (p == 2) {
        if (skipRoot) {
            return utils::Vector(*this * 2.);
        } else {
            return utils::Vector(*this * 1.0/norm(2));
        }
    } else {
        utils::Vector res(static_cast<size_t>(size()));
        utils::Scalar normalized(std::pow(norm(), static_cast<unsigned int>(p-1)));
        for (unsigned int i=0; i< static_cast<unsigned int>(size()); ++i) {
            res[i] = (*this)[i] * std::pow(fabs((*this)[i]), static_cast<unsigned int>(p - 2));
        }
        res /= normalized;
        if (skipRoot) {
            utils::Error::raise("skip root not implemented for p > 2");
        }
        return res;
    }
}

void utils::Vector::operator=(
    const utils::Vector &other)
{
    this->RigidBodyDynamics::Math::VectorNd::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void utils::Vector::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_dynamic::operator=(other);
}

void utils::Vector::operator=(
    const casadi::MX &other)
{
    this->MX_Xd_dynamic::operator=(other);
}

#endif
