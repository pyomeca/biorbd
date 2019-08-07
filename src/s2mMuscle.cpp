#define BIORBD_API_EXPORTS
#include "s2mMuscle.h"

#include "s2mError.h"
#include "s2mMusclePathChangers.h"
#include "s2mMuscleStateDynamics.h"
#include "s2mMuscleStateDynamicsBuchanan.h"
#include "s2mGenCoord.h"

s2mMuscle::s2mMuscle(const s2mString& name,
                     const s2mMuscleGeometry& g,
                     const s2mMuscleCaracteristics& c,
                     const s2mMusclePathChangers& w,
                     const s2mMuscleStateDynamics& s) :
    s2mMuscleCompound(name,w),
    m_position(g),
    m_caract(c)
{
    setState(s);

    s2mError::s2mAssert(w.nbWraps()!=1, "Multiple wrapping objects is not implemented yet");
}

s2mMuscle::s2mMuscle(const s2mMuscle &m) :
    s2mMuscleCompound (m)
{
    this->m_position = m.m_position;
    this->m_caract = m.m_caract;
    this->m_state = m.m_state;
}

s2mMuscle::~s2mMuscle()
{
    //dtor
}

void s2mMuscle::updateOrientations(s2mJoints &m, const s2mGenCoord &Q, int updateKin){
    // Update de la position des insertions et origines
    m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);
}
void s2mMuscle::updateOrientations(s2mJoints &m, const s2mGenCoord &Q, const s2mGenCoord &Qdot, int updateKin){
    // Update de la position des insertions et origines
    m_position.updateKinematics(m,&Q,&Qdot,m_caract,m_pathChanger,updateKin);
}
void s2mMuscle::updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix &jacoPointsInGlobal){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,nullptr,m_caract);
}
void s2mMuscle::updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, s2mMatrix &jacoPointsInGlobal, const s2mGenCoord &Qdot){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,&Qdot,m_caract);
}

const s2mMuscleGeometry &s2mMuscle::position() const {
    return m_position;
}

double s2mMuscle::length(s2mJoints &m, const s2mGenCoord &Q, int updateKin){
    if (updateKin != 0)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);

    return m_position.length();
}

double s2mMuscle::musculoTendonLength(s2mJoints &m, const s2mGenCoord &Q, int updateKin){
    if (updateKin != 0)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);

    return m_position.musculoTendonLength();
}

double s2mMuscle::velocity(s2mJoints &m, const s2mGenCoord &Q, const s2mGenCoord &Qdot, const bool updateKin){
    if (updateKin)
        m_position.updateKinematics(m,&Q,&Qdot,m_caract,m_pathChanger);

    return m_position.velocity();
}

double s2mMuscle::activationDot(const s2mMuscleStateDynamics &s, const bool already){
    return m_state->timeDerivativeActivation(s, caract(), already);
}

const std::vector<s2mNodeMuscle> &s2mMuscle::musclesPointsInGlobal(s2mJoints &m, const s2mGenCoord &Q,const bool updateKin){
    if (updateKin)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger);

    return m_position.musclesPointsInGlobal();
}


void s2mMuscle::forceIsoMax(double val){
    m_caract.setForceIsoMax(val);
}


const s2mMuscleCaracteristics& s2mMuscle::caract() const {
    return m_caract;
}
void s2mMuscle::setPosition(const s2mMuscleGeometry &val) {
    m_position = val;
}
void s2mMuscle::setCaract(const s2mMuscleCaracteristics &val) {
    m_caract = val;
}

// Get and set
void s2mMuscle::setState(const s2mMuscleStateDynamics &s){
    if (dynamic_cast<const s2mMuscleStateDynamicsBuchanan*> (&s)){
        m_state = new s2mMuscleStateDynamicsBuchanan();
    }
    else if (dynamic_cast<const s2mMuscleStateDynamics*> (&s)){
        m_state = new s2mMuscleStateDynamics();
    }
    *m_state = s;
}
const s2mMuscleStateDynamics& s2mMuscle::state() const {
    return *m_state;
}
s2mMuscleStateDynamics& s2mMuscle::state_nonConst() const {
    return *m_state;
}
