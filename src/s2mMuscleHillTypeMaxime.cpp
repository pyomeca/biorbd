#include "../include/s2mMuscleHillTypeMaxime.h"

s2mMuscleHillTypeMaxime::s2mMuscleHillTypeMaxime(const s2mMuscleGeometry& g,
                        const s2mMuscleCaracteristics& c,
                        const s2mMusclePathChangers & w,
                        const s2mMuscleStateActual & s) :
    s2mMuscleHillType(g,c,w,s),
    m_muscleGain(1),
    m_caractMaxime(m_caract)
{
    setType();
    if (caract().forceIsoMax() != 0)
        m_caractMaxime.bruteSetForceIsoMax(caract().forceIsoMax());
}
s2mMuscleHillTypeMaxime::s2mMuscleHillTypeMaxime(const s2mString& n,
                        const s2mMuscleGeometry& g,
                        const s2mMuscleCaracteristics& c,
                        const s2mMusclePathChangers & w,
                        const s2mMuscleStateActual & s) :
    s2mMuscleHillType(n,g,c,w,s),
    m_muscleGain(1),
    m_caractMaxime(m_caract)
{
    setType();
    if (caract().forceIsoMax() != 0)
        m_caractMaxime.bruteSetForceIsoMax(caract().forceIsoMax());
}

s2mMuscleStateActual s2mMuscleHillTypeMaxime::excitationNorm(s2mMuscleStateActual EMG){

    // L'EMG norm doit etre setté d'une facon différente que s2mMuscleHillType
    EMG.excitationNorm(caract().stateMax());
    EMG.setExcitationNorm(x0() * (exp(x1()*EMG.excitationNorm()) -1) / (exp(x1())-1));
    return EMG;
}

void s2mMuscleHillTypeMaxime::computeForce(const s2mMuscleStateActual &EMG){

    double force;
    if (caractMaxime().isForceMaxSet()){
        force = m_muscleGain * caract().forceIsoMax() * (EMG.activation()*m_FlCE*m_FvCE + m_FlPE + m_damping);
    }
    else{
        force = m_muscleGain * caract().PCSA() * EMG.activation();
    }
    m_force[0]->setForce(m_position, force); // origine vers le deuxieme point
    m_force[1]->setForce(m_position, force); // insertion vers l'avant-dernier point


}

void s2mMuscleHillTypeMaxime::forceIsoMax(double val){
    m_caractMaxime.setForceIsoMax(val);
    m_muscleGain = 1;
}




// MUSCLE CARACTERISTICS
s2mMuscleHillTypeMaxime::s2mMuscleCaracteristicsMaxime::s2mMuscleCaracteristicsMaxime(const s2mMuscleCaracteristics& c) :
    s2mMuscleCaracteristics(c),
    m_isForceMaxSet(false)
{

}

s2mMuscleHillTypeMaxime::s2mMuscleCaracteristicsMaxime::s2mMuscleCaracteristicsMaxime(const double &optLength,
                        const double &fmax,
                        const double &PCSA,
                        const double &tendonSlackLength,
                        const double &pennAngle,
                        const s2mMuscleStateMax *stateMax,
                        const double tauAct,
                        const double tauDeact,
                        const double &minAct
                        ) :
    s2mMuscleCaracteristics(optLength, fmax, PCSA, tendonSlackLength, pennAngle, stateMax, tauAct, tauDeact, minAct),
    m_isForceMaxSet(false)
{
}

void s2mMuscleHillTypeMaxime::s2mMuscleCaracteristicsMaxime::setForceIsoMax(const double& muscleGain){
    m_fIsoMax = muscleGain*PCSA();
    m_isForceMaxSet = true;
}

void s2mMuscleHillTypeMaxime::s2mMuscleCaracteristicsMaxime::bruteSetForceIsoMax(const double& forceMax){
    m_fIsoMax = forceMax;
    m_isForceMaxSet = true;
}
