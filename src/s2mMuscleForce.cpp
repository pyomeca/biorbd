#include "../include/s2mMuscleForce.h"

s2mMuscleForce::s2mMuscleForce()
{
    //ctor
    setForce(Eigen::Vector3d(0,0,0));
}
s2mMuscleForce::s2mMuscleForce(const double &v1, const double &v2, const double &v3){
    setForce(v1,v2,v3);
}
s2mMuscleForce::s2mMuscleForce(const s2mMuscleGeometry& geo, const double& f){
    setForce(geo, f);
}


s2mMuscleForce::s2mMuscleForce(const Eigen::Vector3d &V) :
    Eigen::Vector3d(V)
{
    //ctor
    //setForce(V); // Appel récurrent
}

s2mMuscleForce::~s2mMuscleForce()
{
    //dtor
}



void s2mMuscleForce::setForce(const s2mMuscleGeometry& geo, const double& f){
    // Trouver le vecteur directeur
    Eigen::Vector3d V(geo.insertionInGlobal() - geo.originInGlobal());
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*f;

    // Stocker cette valeur
    setForce(V);
}
void s2mMuscleForce::setForce(const double& v1, const double& v2, const double& v3){
    setForce(Eigen::Vector3d(v1, v2, v3));
}

void s2mMuscleForce::setForce(const Eigen::Vector3d &V){
    *this = V;
    computeNorm();
}

void s2mMuscleForce::computeNorm(){
    m_force = (*this).norm();
}
