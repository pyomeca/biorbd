#define BIORBD_API_EXPORTS
#include "Muscles/Force.h"

#include "Utils/Node3d.h"
#include "Muscles/Geometry.h"

biorbd::muscles::Force::Force() :
    Eigen::Vector3d ()
{

}

biorbd::muscles::Force::Force(
        double x,
        double y,
        double z) :
    Eigen::Vector3d (x, y, z)
{

}

// Shallow Copy
biorbd::muscles::Force::Force(const biorbd::muscles::Force &force) :
    Eigen::Vector3d (force)
{

}

biorbd::muscles::Force::Force(const biorbd::utils::Node3d &force) :
    Eigen::Vector3d (force)
{

}

biorbd::muscles::Force::Force(const biorbd::muscles::Geometry& geo,
        double vectorNorm) :
    Eigen::Vector3d()
{
    setForceFromMuscleGeometry(geo, vectorNorm);
}

biorbd::muscles::Force::~Force()
{

}

void biorbd::muscles::Force::setForceFromMuscleGeometry(
        const biorbd::muscles::Geometry& geo,
        double vectorNorm)
{
    *this = geo.insertionInGlobal() - geo.originInGlobal();
    *this /= this->norm() * vectorNorm;
}
