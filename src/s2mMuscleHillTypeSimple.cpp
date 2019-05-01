#define BIORBD_API_EXPORTS
#include "../include/s2mMuscleHillTypeSimple.h"


double s2mMuscleHillTypeSimple::multiplyCaractByActivationAndForce(const s2mMuscleStateActual &EMG){
    return caract().forceIsoMax() * (EMG.activation());
}

std::vector<std::shared_ptr<s2mMuscleForce> > s2mMuscleHillTypeSimple::force(const s2mMuscleStateActual &EMG){
    // Combiner les forces
    computeForce(EMG);
    return m_force;
}
