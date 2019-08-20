#define BIORBD_API_EXPORTS
#include "Muscles/Force.h"

biorbd::muscles::Force::Force(double x, double y, double z) :
    Eigen::Vector3d (x, y, z)
{
    computeNorm();
}
biorbd::muscles::Force::Force(
        const biorbd::muscles::Geometry& geo,
        double force)
{
    setForce(geo, force);
}


biorbd::muscles::Force::Force(const biorbd::utils::Node3d &force)
{
    //ctor
    setForce(force); // Appel récurrent
}

double biorbd::muscles::Force::norme() const
{
    return m_force;
}

const biorbd::muscles::Force &biorbd::muscles::Force::directionVector() const
{
    return *this;
}



void biorbd::muscles::Force::setForce(const biorbd::muscles::Geometry& geo, double force)
{
    // Trouver le vecteur directeur
    biorbd::utils::Node3d V(geo.insertionInGlobal() - geo.originInGlobal());
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    setForce(V);
}
void biorbd::muscles::Force::setForce(double x, double y, double z)
{
    setForce(biorbd::utils::Node3d(x, y, z));
}

void biorbd::muscles::Force::setForce(const biorbd::utils::Node3d &force)
{
    *this = force;
    computeNorm();
}

void biorbd::muscles::Force::computeNorm(){
    m_force = (*this).norm();
}
