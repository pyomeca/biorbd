#define BIORBD_API_EXPORTS
#include "../include/s2mMuscle.h"


s2mMuscle::s2mMuscle(const s2mString& name,
                     const s2mMuscleGeometry& g,
                     const s2mMuscleCaracteristics& c,
                     const s2mMusclePathChangers& w,
                     const s2mMuscleStateActual& s) :
    s2mMuscleCompound(name,w),
    m_position(g),
    m_caract(c)
{
    setState(s);

    s2mError::s2mAssert(w.nbWraps()!=1, "Multiple wrapping objects is not implemented yet");
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
void s2mMuscle::updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, Eigen::MatrixXd& jacoPointsInGlobal){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,nullptr,m_caract);
}
void s2mMuscle::updateOrientations(std::vector<s2mNodeMuscle>& musclePointsInGlobal, Eigen::MatrixXd& jacoPointsInGlobal, const s2mGenCoord &Qdot){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,&Qdot,m_caract);
}

s2mMuscleGeometry s2mMuscle::position() const {
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

double s2mMuscle::activationDot(const s2mMuscleStateActual &s, const bool already){
    return m_state->timeDerivativeActivation(s, caract(), already);
}

std::vector<s2mNodeMuscle> s2mMuscle::musclesPointsInGlobal(s2mJoints &m, const s2mGenCoord &Q,const bool updateKin){
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
void s2mMuscle::setState(const s2mMuscleStateActual &s){
    if (dynamic_cast<const s2mMuscleStateActualBuchanan*> (&s)){
        m_state = new s2mMuscleStateActualBuchanan();
    }
    else if (dynamic_cast<const s2mMuscleStateActual*> (&s)){
        m_state = new s2mMuscleStateActual();
    }
    *m_state = s;
}
const s2mMuscleStateActual& s2mMuscle::state() const {
    return *m_state;
}
s2mMuscleStateActual& s2mMuscle::state_nonConst() const {
    return *m_state;
}
