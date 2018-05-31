#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleMeshTransverse.h"

s2mMuscleMeshTransverse::s2mMuscleMeshTransverse()
{
    setType();
    m_nbTrans = 0;
}

s2mMuscleMeshTransverse::~s2mMuscleMeshTransverse()
{

}

// Muscles components
s2mMuscleStateActual s2mMuscleMeshTransverse::dynamicMuscleActivation() {
    s2mMuscleStateActual S0;
    return S0;
}
s2mMuscleStateActual s2mMuscleMeshTransverse::computeDynamicMuscleActivation() {
    s2mMuscleStateActual S0;
    return S0;
}
s2mMuscleStateActual s2mMuscleMeshTransverse::dynamicMuscleContraction() {
    s2mMuscleStateActual S0;
    return S0;
}
s2mMuscleStateActual s2mMuscleMeshTransverse::computeDynamicMuscleContraction() {
    s2mMuscleStateActual S0;
    return S0;
}
s2mMuscleGeometry s2mMuscleMeshTransverse::updateMuscleGeometry() {
    s2mMuscleGeometry S0;
    return S0;
}

s2mMuscleForce s2mMuscleMeshTransverse::computeForce() {
    s2mMuscleForce dummy;
    return dummy;
}

// Méthodes locales
void s2mMuscleMeshTransverse::setNbTransverseElement(const unsigned int n){
    m_nbTrans = n;

}

