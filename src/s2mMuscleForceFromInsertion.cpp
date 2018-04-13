#include "../include/s2mMuscleForceFromInsertion.h"

s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion() :
    s2mMuscleForce(){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(const double &v1, const double &v2, const double &v3) :
    s2mMuscleForce(v1,v2,v3){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(const s2mMuscleGeometry& geo, const double& f) :
    s2mMuscleForce(geo, f){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(const Eigen::Vector3d &V) :
    s2mMuscleForce(V){}

s2mMuscleForceFromInsertion::~s2mMuscleForceFromInsertion(){}



void s2mMuscleForceFromInsertion::setForce(const s2mMuscleGeometry& geo, const double& f){
    // Trouver le vecteur directeur
    std::vector<s2mNodeMuscle> tp_via = geo.musclesPointsInGlobal();
    Eigen::Vector3d V ( tp_via[tp_via.size()-2] - tp_via[tp_via.size()-1] );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*f;

    // Stocker cette valeur
    s2mMuscleForce::setForce(V);
}
