#define BIORBD_API_EXPORTS
#include "Muscles/Muscle.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/PathChangers.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/StateDynamicsBuchanan.h"

biorbd::muscles::Muscle::Muscle(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers& w,
        const biorbd::muscles::StateDynamics& s) :
    biorbd::muscles::Compound(name,w),
    m_position(g),
    m_caract(c)
{
    setState(s);

    biorbd::utils::Error::error(w.nbWraps()!=1, "Multiple wrapping objects is not implemented yet");
}

biorbd::muscles::Muscle::Muscle(const biorbd::muscles::Muscle &muscle) :
    biorbd::muscles::Compound (muscle)
{
    this->m_position = muscle.m_position;
    this->m_caract = muscle.m_caract;
    this->m_state = muscle.m_state;
}

biorbd::muscles::Muscle::~Muscle()
{
    //dtor
}

void biorbd::muscles::Muscle::updateOrientations(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        int updateKin){
    // Update de la position des insertions et origines
    m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);
}
void biorbd::muscles::Muscle::updateOrientations(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        int updateKin){
    // Update de la position des insertions et origines
    m_position.updateKinematics(m,&Q,&Qdot,m_caract,m_pathChanger,updateKin);
}
void biorbd::muscles::Muscle::updateOrientations(
        std::vector<biorbd::muscles::MuscleNode>& musclePointsInGlobal,
        biorbd::utils::Matrix &jacoPointsInGlobal){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,nullptr,m_caract);
}
void biorbd::muscles::Muscle::updateOrientations(
        std::vector<biorbd::muscles::MuscleNode>& musclePointsInGlobal,
        biorbd::utils::Matrix &jacoPointsInGlobal,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot){
    // Update de la position des insertions et origines
    m_position.updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,&Qdot,m_caract);
}

const biorbd::muscles::Geometry &biorbd::muscles::Muscle::position() const {
    return m_position;
}

double biorbd::muscles::Muscle::length(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        int updateKin){
    if (updateKin != 0)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);

    return m_position.length();
}

double biorbd::muscles::Muscle::musculoTendonLength(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        int updateKin){
    if (updateKin != 0)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger,updateKin);

    return m_position.musculoTendonLength();
}

double biorbd::muscles::Muscle::velocity(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        bool updateKin){
    if (updateKin)
        m_position.updateKinematics(m,&Q,&Qdot,m_caract,m_pathChanger);

    return m_position.velocity();
}

double biorbd::muscles::Muscle::activationDot(const biorbd::muscles::StateDynamics &s, bool already){
    return m_state->timeDerivativeActivation(s, caract(), already);
}

const std::vector<biorbd::muscles::MuscleNode> &biorbd::muscles::Muscle::musclesPointsInGlobal(
        biorbd::rigidbody::Joints &m,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin){
    if (updateKin)
        m_position.updateKinematics(m,&Q,nullptr,m_caract,m_pathChanger);

    return m_position.musclesPointsInGlobal();
}


void biorbd::muscles::Muscle::forceIsoMax(double val){
    m_caract.setForceIsoMax(val);
}


const biorbd::muscles::Caracteristics& biorbd::muscles::Muscle::caract() const {
    return m_caract;
}
void biorbd::muscles::Muscle::setPosition(const biorbd::muscles::Geometry &val) {
    m_position = val;
}
void biorbd::muscles::Muscle::setCaract(const biorbd::muscles::Caracteristics &val) {
    m_caract = val;
}

// Get and set
void biorbd::muscles::Muscle::setState(const biorbd::muscles::StateDynamics &s){
    if (dynamic_cast<const biorbd::muscles::StateDynamicsBuchanan*> (&s)){
        m_state = new biorbd::muscles::StateDynamicsBuchanan();
    }
    else if (dynamic_cast<const biorbd::muscles::StateDynamics*> (&s)){
        m_state = new biorbd::muscles::StateDynamics();
    }
    *m_state = s;
}
const biorbd::muscles::StateDynamics& biorbd::muscles::Muscle::state() const {
    return *m_state;
}
biorbd::muscles::StateDynamics& biorbd::muscles::Muscle::state_nonConst() const {
    return *m_state;
}
