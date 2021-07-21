#define BIORBD_API_EXPORTS
#include "Utils/Scalar.h"

#ifdef BIORBD_USE_CASADI_MATH
#include "Utils/Error.h"

using namespace BIORBD_NAMESPACE;

utils::Scalar::Scalar() :
    RigidBodyDynamics::Math::Scalar ()
{

}

utils::Scalar::Scalar(
    double val) :
    RigidBodyDynamics::Math::Scalar (val)
{

}

utils::Scalar::Scalar(
    const casadi::MX &val) :
    RigidBodyDynamics::Math::Scalar (val)
{
    utils::Error::check(
        val.rows() == 1 && val.columns() == 1,
        "Scalar must be a MX 1x1");
}

#endif
