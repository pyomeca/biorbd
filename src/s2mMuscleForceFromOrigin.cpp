#include "../include/s2mMuscleForceFromOrigin.h"

s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin() :
    s2mMuscleForce(){}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(const double &v1, const double &v2, const double &v3) :
    s2mMuscleForce(v1,v2,v3){}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(const s2mMuscleGeometry& geo, const double& f) :
    s2mMuscleForce(geo, f){}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(const Eigen::Vector3d &V) :
    s2mMuscleForce(V){}

s2mMuscleForceFromOrigin::~s2mMuscleForceFromOrigin(){}



void s2mMuscleForceFromOrigin::setForce(const s2mMuscleGeometry& geo, const double& f){
    // Trouver le vecteur directeur
    // Trouver le vecteur directeur
    std::vector<s2mNodeMuscle> tp_via = geo.musclesPointsInGlobal();
    Eigen::Vector3d V ( tp_via[1] - tp_via[0]  );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*f;

    // Stocker cette valeur
    s2mMuscleForce::setForce(V);
}
