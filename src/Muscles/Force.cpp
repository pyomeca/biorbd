#define BIORBD_API_EXPORTS
#include "Muscles/Force.h"

#include "Muscles/Geometry.h"

biorbd::muscles::Force::Force() :
    biorbd::utils::Vector3d ()
{

}

biorbd::muscles::Force::Force(
        double x,
        double y,
        double z) :
    biorbd::utils::Vector3d (x, y, z)
{

}

// Shallow Copy
biorbd::muscles::Force::Force(
        const biorbd::muscles::Force &other) :
    biorbd::utils::Vector3d (other)
{

}

biorbd::muscles::Force::Force(
        const biorbd::utils::Vector3d& other) :
    biorbd::utils::Vector3d (other)
{

}

biorbd::muscles::Force::Force(
        const biorbd::muscles::Geometry& geo,
        double norm) :
    biorbd::utils::Vector3d()
{
    setForceFromMuscleGeometry(geo, norm);
}

biorbd::muscles::Force::~Force()
{

}

biorbd::muscles::Force biorbd::muscles::Force::DeepCopy() const
{
    return *this;
}

void biorbd::muscles::Force::DeepCopy(const biorbd::muscles::Force &other)
{
    *this = other;
}

void biorbd::muscles::Force::setForceFromMuscleGeometry(
        const biorbd::muscles::Geometry& geo,
        double norm)
{
    *this = geo.insertionInGlobal() - geo.originInGlobal();
    *this /= this->norm() * norm;
}

biorbd::muscles::Force &biorbd::muscles::Force::operator=(const biorbd::muscles::Force &other)
{
    if (this == &other)
        return *this;

    this->Eigen::Vector3d::operator=(other);
    return *this;
}
