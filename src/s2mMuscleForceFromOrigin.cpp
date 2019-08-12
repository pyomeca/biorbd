#define BIORBD_API_EXPORTS
#include "s2mMuscleForceFromOrigin.h"

s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin() :
    s2mMuscleForce()
{

}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(double x, double y, double z) :
    s2mMuscleForce(x,y,z)
{

}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(
        const s2mMuscleGeometry& geo,
        double force) :
    s2mMuscleForce(geo, force)
{

}
s2mMuscleForceFromOrigin::s2mMuscleForceFromOrigin(const Eigen::Vector3d &force) :
    s2mMuscleForce(force)
{

}

s2mMuscleForceFromOrigin::~s2mMuscleForceFromOrigin()
{

}



void s2mMuscleForceFromOrigin::setForce(const s2mMuscleGeometry& geo, double force){
    // Trouver le vecteur directeur
    // Trouver le vecteur directeur
    std::vector<s2mNodeMuscle> tp_via = geo.musclesPointsInGlobal();
    Eigen::Vector3d V ( tp_via[1] - tp_via[0]  );
    V = V/V.norm();

    // Agrandir le vecteur selon sa vraie grandeur (fonction de la force)
    V = V*force;

    // Stocker cette valeur
    s2mMuscleForce::setForce(V);
}
