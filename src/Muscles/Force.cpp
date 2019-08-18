#define BIORBD_API_EXPORTS
#include "Muscles/Force.h"

biorbd::muscles::Force::Force()
{
    //ctor
    setForce(Eigen::Vector3d(0,0,0));
}
biorbd::muscles::Force::Force(double x, double y, double z){
    setForce(x,y,z);
}
biorbd::muscles::Force::Force(
        const biorbd::muscles::Geometry& geo,
        double force){
    setForce(geo, force);
}


biorbd::muscles::Force::Force(const Eigen::Vector3d &force) :
    Eigen::Vector3d(force)
{
    //ctor
    //setForce(V); // Appel récurrent
}

double biorbd::muscles::Force::norme() const
{
    return m_force;
}

const Eigen::Vector3d &biorbd::muscles::Force::directionVector() const
{
    return *this;
}



void biorbd::muscles::Force::setForce(const biorbd::muscles::Geometry& geo, double force){
    // Trouver le vecteur directeur
    Eigen::Vector3d V(geo.insertionInGlobal() - geo.originInGlobal());
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    setForce(V);
}
void biorbd::muscles::Force::setForce(double x, double y, double z){
    setForce(Eigen::Vector3d(x, y, z));
}

void biorbd::muscles::Force::setForce(const Eigen::Vector3d &force){
    *this = force;
    computeNorm();
}

void biorbd::muscles::Force::computeNorm(){
    m_force = (*this).norm();
}
