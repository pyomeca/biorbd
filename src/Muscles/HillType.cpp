#define BIORBD_API_EXPORTS
#include "Muscles/HillType.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/ForceFromOrigin.h"
#include "Muscles/ForceFromInsertion.h"
#include "Muscles/Caracteristics.h"
#include "Muscles/Geometry.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::HillType::HillType() :
    biorbd::muscles::Muscle(),
    m_damping(std::make_shared<double>()),
    m_FlCE(std::make_shared<double>()),
    m_FlPE(std::make_shared<double>()),
    m_FvCE(std::make_shared<double>()),
    m_cste_FlCE_1(std::make_shared<double>(0.15)),
    m_cste_FlCE_2(std::make_shared<double>(0.45)),
    m_cste_FvCE_1(std::make_shared<double>(1)),
    m_cste_FvCE_2(std::make_shared<double>(-.33/2 * *m_cste_FvCE_1/(1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<double>(10.0)),
    m_cste_FlPE_2(std::make_shared<double>(5.0)),
    m_cste_forceExcentriqueMultiplier(std::make_shared<double>(1.8)),
    m_cste_damping(std::make_shared<double>(0.1)),
    m_cste_vitesseRaccourMax(std::make_shared<double>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract) :
    biorbd::muscles::Muscle(name,geometry,caract),
    m_damping(std::make_shared<double>()),
    m_FlCE(std::make_shared<double>()),
    m_FlPE(std::make_shared<double>()),
    m_FvCE(std::make_shared<double>()),
    m_cste_FlCE_1(std::make_shared<double>(0.15)),
    m_cste_FlCE_2(std::make_shared<double>(0.45)),
    m_cste_FvCE_1(std::make_shared<double>(1)),
    m_cste_FvCE_2(std::make_shared<double>(-.33/2 * *m_cste_FvCE_1/(1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<double>(10.0)),
    m_cste_FlPE_2(std::make_shared<double>(5.0)),
    m_cste_forceExcentriqueMultiplier(std::make_shared<double>(1.8)),
    m_cste_damping(std::make_shared<double>(0.1)),
    m_cste_vitesseRaccourMax(std::make_shared<double>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract,
        const biorbd::muscles::StateDynamics &dynamicState) :
    biorbd::muscles::Muscle(name,geometry,caract, dynamicState),
    m_damping(std::make_shared<double>()),
    m_FlCE(std::make_shared<double>()),
    m_FlPE(std::make_shared<double>()),
    m_FvCE(std::make_shared<double>()),
    m_cste_FlCE_1(std::make_shared<double>(0.15)),
    m_cste_FlCE_2(std::make_shared<double>(0.45)),
    m_cste_FvCE_1(std::make_shared<double>(1)),
    m_cste_FvCE_2(std::make_shared<double>(-.33/2 * *m_cste_FvCE_1/(1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<double>(10.0)),
    m_cste_FlPE_2(std::make_shared<double>(5.0)),
    m_cste_forceExcentriqueMultiplier(std::make_shared<double>(1.8)),
    m_cste_damping(std::make_shared<double>(0.1)),
    m_cste_vitesseRaccourMax(std::make_shared<double>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
        const biorbd::utils::String &name,
        const biorbd::muscles::Geometry &geometry,
        const biorbd::muscles::Caracteristics &caract,
        const biorbd::muscles::PathChangers &pathChangers) :
    biorbd::muscles::Muscle(name,geometry,caract,pathChangers),
    m_damping(std::make_shared<double>()),
    m_FlCE(std::make_shared<double>()),
    m_FlPE(std::make_shared<double>()),
    m_FvCE(std::make_shared<double>()),
    m_cste_FlCE_1(std::make_shared<double>(0.15)),
    m_cste_FlCE_2(std::make_shared<double>(0.45)),
    m_cste_FvCE_1(std::make_shared<double>(1)),
    m_cste_FvCE_2(std::make_shared<double>(-.33/2 * *m_cste_FvCE_1/(1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<double>(10.0)),
    m_cste_FlPE_2(std::make_shared<double>(5.0)),
    m_cste_forceExcentriqueMultiplier(std::make_shared<double>(1.8)),
    m_cste_damping(std::make_shared<double>(0.1)),
    m_cste_vitesseRaccourMax(std::make_shared<double>(10.0))
{
    setType();
}
biorbd::muscles::HillType::HillType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Caracteristics& caract,
        const biorbd::muscles::PathChangers & wrapper,
        const biorbd::muscles::StateDynamics & state) :
    biorbd::muscles::Muscle(name,geometry,caract,wrapper,state),
    m_damping(std::make_shared<double>()),
    m_FlCE(std::make_shared<double>()),
    m_FlPE(std::make_shared<double>()),
    m_FvCE(std::make_shared<double>()),
    m_cste_FlCE_1(std::make_shared<double>(0.15)),
    m_cste_FlCE_2(std::make_shared<double>(0.45)),
    m_cste_FvCE_1(std::make_shared<double>(1)),
    m_cste_FvCE_2(std::make_shared<double>(-.33/2 * *m_cste_FvCE_1/(1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<double>(10.0)),
    m_cste_FlPE_2(std::make_shared<double>(5.0)),
    m_cste_forceExcentriqueMultiplier(std::make_shared<double>(1.8)),
    m_cste_damping(std::make_shared<double>(0.1)),
    m_cste_vitesseRaccourMax(std::make_shared<double>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
        const biorbd::muscles::Muscle &muscle) :
    biorbd::muscles::Muscle (muscle)
{
    const biorbd::muscles::HillType & m_tp(dynamic_cast<const biorbd::muscles::HillType &>(muscle));
    m_damping = m_tp.m_damping;
    m_FlCE = m_tp.m_FlCE;
    m_FlPE = m_tp.m_FlPE;
    m_FvCE = m_tp.m_FvCE;
    m_cste_FlCE_1 = m_tp.m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp.m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp.m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp.m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp.m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp.m_cste_FlPE_2;
    m_cste_forceExcentriqueMultiplier = m_tp.m_cste_forceExcentriqueMultiplier;
    m_cste_damping = m_tp.m_cste_damping;
    m_cste_vitesseRaccourMax = m_tp.m_cste_vitesseRaccourMax;
}

biorbd::muscles::HillType::HillType(
        const std::shared_ptr<biorbd::muscles::Muscle> muscle) :
    biorbd::muscles::Muscle (muscle)
{
    const std::shared_ptr<biorbd::muscles::HillType> m_tp(std::dynamic_pointer_cast<biorbd::muscles::HillType>(muscle));
    biorbd::utils::Error::check(m_tp != nullptr, "Muscle must be of a Hill Type");
    m_damping = m_tp->m_damping;
    m_FlCE = m_tp->m_FlCE;
    m_FlPE = m_tp->m_FlPE;
    m_FvCE = m_tp->m_FvCE;
    m_cste_FlCE_1 = m_tp->m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp->m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp->m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp->m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp->m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp->m_cste_FlPE_2;
    m_cste_forceExcentriqueMultiplier = m_tp->m_cste_forceExcentriqueMultiplier;
    m_cste_damping = m_tp->m_cste_damping;
    m_cste_vitesseRaccourMax = m_tp->m_cste_vitesseRaccourMax;
}

biorbd::muscles::HillType biorbd::muscles::HillType::DeepCopy() const
{
    biorbd::muscles::HillType copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::HillType::DeepCopy(const biorbd::muscles::HillType &other)
{
    biorbd::muscles::Muscle::DeepCopy(other);
    *m_damping = *other.m_damping;
    *m_FlCE = *other.m_FlCE;
    *m_FlPE = *other.m_FlPE;
    *m_FvCE = *other.m_FvCE;
    *m_cste_FlCE_1 = *other.m_cste_FlCE_1;
    *m_cste_FlCE_2 = *other.m_cste_FlCE_2;
    *m_cste_FvCE_1 = *other.m_cste_FvCE_1;
    *m_cste_FvCE_2 = *other.m_cste_FvCE_2;
    *m_cste_FlPE_1 = *other.m_cste_FlPE_1;
    *m_cste_FlPE_2 = *other.m_cste_FlPE_2;
    *m_cste_forceExcentriqueMultiplier = *other.m_cste_forceExcentriqueMultiplier;
    *m_cste_damping = *other.m_cste_damping;
    *m_cste_vitesseRaccourMax = *other.m_cste_vitesseRaccourMax;
}

const std::vector<std::shared_ptr<biorbd::muscles::Force> > &biorbd::muscles::HillType::force(
        const biorbd::muscles::StateDynamics& emg){
    // Calculer chacune les forces dans chaque éléments
    computeFvCE();
    computeFlCE(emg);
    computeFlPE();
    computeDamping();

    // Combiner les forces
    computeForce(emg);
    return *m_force;
}

const std::vector<std::shared_ptr<biorbd::muscles::Force> > &biorbd::muscles::HillType::force(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &Qdot,
        const biorbd::muscles::StateDynamics &emg,
        int updateKin)
{
    // Update de la configuration
    if (updateKin == 1)
        updateOrientations(model,Q,Qdot,false);
    else if (updateKin == 2)
        updateOrientations(model,Q,Qdot,true);
    else
        biorbd::utils::Error::check(updateKin == 0, "Wrong level of update in force function");

    // Calculs
    return force(emg);
}

const std::vector<std::shared_ptr<biorbd::muscles::Force> > &biorbd::muscles::HillType::force(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::muscles::StateDynamics &,
        int)
{
    biorbd::utils::Error::raise("Hill type needs velocity");
    return *m_force; // Will never reach here
}

double biorbd::muscles::HillType::FlCE(
        const biorbd::muscles::StateDynamics &EMG)
{
    computeFlCE(EMG);
    return *m_FlCE;
}

double biorbd::muscles::HillType::FlPE()
{
    computeFlPE();
    return *m_FlPE;
}

double biorbd::muscles::HillType::FvCE()
{
    computeFvCE();
    return *m_FvCE;
}

double biorbd::muscles::HillType::damping()
{
    computeDamping();
    return *m_damping;
}

void biorbd::muscles::HillType::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::HILL;
}

void biorbd::muscles::HillType::computeDamping(){
    *m_damping = position().velocity() / (*m_cste_vitesseRaccourMax * m_caract->optimalLength()) * *m_cste_damping;
}

void biorbd::muscles::HillType::computeFlCE(const biorbd::muscles::StateDynamics &emg){
    *m_FlCE = exp( -pow(( position().length() / m_caract->optimalLength() / (*m_cste_FlCE_1*(1-emg.activation())+1) -1 ), 2)
                  /
                  *m_cste_FlCE_2   );
}

void biorbd::muscles::HillType::computeFvCE(){
    // La relation est différente si la vitesse < 0  ou > 0
    double v = m_position->velocity();
    if (v<=0)
        *m_FvCE = (1-abs(v) / *m_cste_vitesseRaccourMax) /
            (1+abs(v) / *m_cste_vitesseRaccourMax / *m_cste_FvCE_1);
    else
        *m_FvCE = (1-1.33*v / *m_cste_vitesseRaccourMax / *m_cste_FvCE_2) /
                   (1-v / *m_cste_vitesseRaccourMax / *m_cste_FvCE_2);
}

void biorbd::muscles::HillType::computeFlPE(){
    if (position().length() > caract().tendonSlackLength())
        *m_FlPE = exp(*m_cste_FlPE_1*(position().length()/caract().optimalLength()-1) - *m_cste_FlPE_2);
    else
        *m_FlPE = 0;
}

double biorbd::muscles::HillType::getForceFromActivation(
        const biorbd::muscles::State &emg)
{
    // Fonction qui permet de modifier la facon dont la multiplication est faite dans computeForce(EMG)
    return caract().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + *m_damping);
}

biorbd::muscles::StateDynamics biorbd::muscles::HillType::normalizeEMG(const biorbd::muscles::StateDynamics &emg){
    biorbd::muscles::StateDynamics emg_out(emg);
    emg_out.excitationNorm(caract().stateMax());
    return emg_out;
}
