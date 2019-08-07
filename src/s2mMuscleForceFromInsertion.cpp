#define BIORBD_API_EXPORTS
#include "s2mMuscleForceFromInsertion.h"

s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion() :
    s2mMuscleForce(){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(double x, double y, double z) :
    s2mMuscleForce(x,y,z){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(const s2mMuscleGeometry& geo, double force) :
    s2mMuscleForce(geo, force){}
s2mMuscleForceFromInsertion::s2mMuscleForceFromInsertion(const Eigen::Vector3d &force) :
    s2mMuscleForce(force){}

s2mMuscleForceFromInsertion::~s2mMuscleForceFromInsertion(){}



void s2mMuscleForceFromInsertion::setForce(const s2mMuscleGeometry& geo, double force){
    // Trouver le vecteur directeur
    std::vector<s2mNodeMuscle> tp_via = geo.musclesPointsInGlobal();
    Eigen::Vector3d V ( tp_via[tp_via.size()-2] - tp_via[tp_via.size()-1] );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    s2mMuscleForce::setForce(V);
}
