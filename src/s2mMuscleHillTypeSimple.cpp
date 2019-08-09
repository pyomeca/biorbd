#define BIORBD_API_EXPORTS
#include "s2mMuscleHillTypeSimple.h"


s2mMuscleHillTypeSimple::s2mMuscleHillTypeSimple(const s2mString &s) :
    s2mMuscleHillType(s)
{
    setType();
}

s2mMuscleHillTypeSimple::s2mMuscleHillTypeSimple(const s2mMuscleGeometry &g,
                                                 const s2mMuscleCaracteristics &c,
                                                 const s2mMusclePathChangers &w,
                                                 const s2mMuscleStateDynamics &s) :
    s2mMuscleHillType(g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeSimple::s2mMuscleHillTypeSimple(const s2mString &n,
                                                 const s2mMuscleGeometry &g,
                                                 const s2mMuscleCaracteristics &c,
                                                 const s2mMusclePathChangers &w,
                                                 const s2mMuscleStateDynamics &s) :
    s2mMuscleHillType(n,g,c,w,s)
{
    setType();
}

s2mMuscleHillTypeSimple::s2mMuscleHillTypeSimple(const s2mMuscle &m) :
    s2mMuscleHillType (m)
{
    setType();
}

s2mMuscleHillTypeSimple::s2mMuscleHillTypeSimple(const std::shared_ptr<s2mMuscle> m) :
    s2mMuscleHillType (m)
{
    setType();
}

s2mMuscleHillTypeSimple::~s2mMuscleHillTypeSimple()
{
    setType();
}

const std::vector<std::shared_ptr<s2mMuscleForce>> &s2mMuscleHillTypeSimple::force(const s2mMuscleStateDynamics &emg){
    // Combiner les forces
    computeForce(emg);
    return m_force;
}

double s2mMuscleHillTypeSimple::multiplyCaractByActivationAndForce(const s2mMuscleStateDynamics &emg){
    return caract().forceIsoMax() * (emg.activation());
}

void s2mMuscleHillTypeSimple::setType()
{
    m_type = "HillSimple";
}
