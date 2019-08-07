#define BIORBD_API_EXPORTS
#include "s2mMuscleForce.h"

s2mMuscleForce::s2mMuscleForce()
{
    //ctor
    setForce(Eigen::Vector3d(0,0,0));
}
s2mMuscleForce::s2mMuscleForce(double x, double y, double z){
    setForce(x,y,z);
}
s2mMuscleForce::s2mMuscleForce(const s2mMuscleGeometry& geo, double force){
    setForce(geo, force);
}


s2mMuscleForce::s2mMuscleForce(const Eigen::Vector3d &force) :
    Eigen::Vector3d(force)
{
    //ctor
    //setForce(V); // Appel récurrent
}

s2mMuscleForce::~s2mMuscleForce()
{
    //dtor
}

double s2mMuscleForce::norme() const
{
    return m_force;
}

const Eigen::Vector3d &s2mMuscleForce::directionVector() const
{
    return *this;
}



void s2mMuscleForce::setForce(const s2mMuscleGeometry& geo, double force){
    // Trouver le vecteur directeur
    Eigen::Vector3d V(geo.insertionInGlobal() - geo.originInGlobal());
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    setForce(V);
}
void s2mMuscleForce::setForce(double x, double y, double z){
    setForce(Eigen::Vector3d(x, y, z));
}

void s2mMuscleForce::setForce(const Eigen::Vector3d &force){
    *this = force;
    computeNorm();
}

void s2mMuscleForce::computeNorm(){
    m_force = (*this).norm();
}
