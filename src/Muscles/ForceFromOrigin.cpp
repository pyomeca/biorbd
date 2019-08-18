#define BIORBD_API_EXPORTS
#include "Muscles/ForceFromOrigin.h"

biorbd::muscles::ForceFromOrigin::ForceFromOrigin() :
    biorbd::muscles::Force()
{

}
biorbd::muscles::ForceFromOrigin::ForceFromOrigin(double x, double y, double z) :
    biorbd::muscles::Force(x,y,z)
{

}
biorbd::muscles::ForceFromOrigin::ForceFromOrigin(
        const biorbd::muscles::Geometry& geo,
        double force) :
    biorbd::muscles::Force(geo, force)
{

}
biorbd::muscles::ForceFromOrigin::ForceFromOrigin(const Eigen::Vector3d &force) :
    biorbd::muscles::Force(force)
{

}

biorbd::muscles::ForceFromOrigin::~ForceFromOrigin()
{

}



void biorbd::muscles::ForceFromOrigin::setForce(const biorbd::muscles::Geometry& geo, double force){
    // Trouver le vecteur directeur
    // Trouver le vecteur directeur
    std::vector<biorbd::muscles::MuscleNode> tp_via = geo.musclesPointsInGlobal();
    Eigen::Vector3d V ( tp_via[1] - tp_via[0]  );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    biorbd::muscles::Force::setForce(V);
}
