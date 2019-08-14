#define BIORBD_API_EXPORTS
#include "Muscles/HillTypeSimple.h"


biorbd::muscles::HillTypeSimple::HillTypeSimple(const biorbd::utils::String &s) :
    biorbd::muscles::HillType(s)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const biorbd::muscles::Geometry &g,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers &w,
        const biorbd::muscles::StateDynamics &s) :
    biorbd::muscles::HillType(g,c,w,s)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(
        const biorbd::utils::String &n,
        const biorbd::muscles::Geometry &g,
        const biorbd::muscles::Caracteristics &c,
        const biorbd::muscles::PathChangers &w,
        const biorbd::muscles::StateDynamics &s) :
    biorbd::muscles::HillType(n,g,c,w,s)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(const biorbd::muscles::Muscle &m) :
    biorbd::muscles::HillType (m)
{
    setType();
}

biorbd::muscles::HillTypeSimple::HillTypeSimple(const std::shared_ptr<biorbd::muscles::Muscle> m) :
    biorbd::muscles::HillType (m)
{
    setType();
}

biorbd::muscles::HillTypeSimple::~HillTypeSimple()
{
    setType();
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>> &biorbd::muscles::HillTypeSimple::force(const biorbd::muscles::StateDynamics &emg){
    // Combiner les forces
    computeForce(emg);
    return m_force;
}

double biorbd::muscles::HillTypeSimple::multiplyCaractByActivationAndForce(const biorbd::muscles::StateDynamics &emg){
    return caract().forceIsoMax() * (emg.activation());
}

void biorbd::muscles::HillTypeSimple::setType()
{
    m_type = "HillSimple";
}
