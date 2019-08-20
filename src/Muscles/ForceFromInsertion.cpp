#define BIORBD_API_EXPORTS
#include "Muscles/ForceFromInsertion.h"

biorbd::muscles::ForceFromInsertion::ForceFromInsertion() :
    biorbd::muscles::Force()
{

}
biorbd::muscles::ForceFromInsertion::ForceFromInsertion(double x, double y, double z) :
    biorbd::muscles::Force(x,y,z)
{

}
biorbd::muscles::ForceFromInsertion::ForceFromInsertion(
        const biorbd::muscles::Geometry& geo,
        double force) :
    biorbd::muscles::Force(geo, force)
{

}
biorbd::muscles::ForceFromInsertion::ForceFromInsertion(const biorbd::utils::Node &force) :
    biorbd::muscles::Force(force)
{

}

biorbd::muscles::ForceFromInsertion::~ForceFromInsertion()
{

}



void biorbd::muscles::ForceFromInsertion::setForce(const biorbd::muscles::Geometry& geo, double force){
    // Trouver le vecteur directeur
    std::vector<biorbd::muscles::MuscleNode> tp_via = geo.musclesPointsInGlobal();
    biorbd::utils::Node V ( tp_via[tp_via.size()-2] - tp_via[tp_via.size()-1] );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    biorbd::muscles::Force::setForce(V);
}
