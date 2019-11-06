#define BIORBD_API_EXPORTS
#include "Muscles/ForceFromOrigin.h"

#include <vector>
#include "Utils/Node3d.h"
#include "Muscles/Geometry.h"

biorbd::muscles::ForceFromOrigin::ForceFromOrigin(double x, double y, double z) :
    biorbd::muscles::Force(x,y,z)
{

}
biorbd::muscles::ForceFromOrigin::ForceFromOrigin(
        const biorbd::muscles::Geometry& geo,
        double vectorNorm) :
    biorbd::muscles::Force()
{
    setForceFromMuscleGeometry(geo, vectorNorm);
}

biorbd::muscles::ForceFromOrigin biorbd::muscles::ForceFromOrigin::DeepCopy() const
{
    biorbd::muscles::ForceFromOrigin copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::ForceFromOrigin::DeepCopy(const biorbd::muscles::ForceFromOrigin &other)
{
    biorbd::muscles::Force::DeepCopy(other);
}


void biorbd::muscles::ForceFromOrigin::setForceFromMuscleGeometry(
        const biorbd::muscles::Geometry& geo,
        double vectorNorm)
{
    //Find the direction vector
    const std::vector<biorbd::utils::Node3d>& tp_via = geo.musclesPointsInGlobal();
    *this = tp_via[1] - tp_via[0];
    *this /= this->norm();
    *this *= vectorNorm;
}
