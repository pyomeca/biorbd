#define BIORBD_API_EXPORTS
#include "Muscles/HillType.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/ForceFromOrigin.h"
#include "Muscles/ForceFromInsertion.h"

biorbd::muscles::HillType::HillType(const biorbd::utils::String& name) :
    biorbd::muscles::Muscle(name),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10.0),
    m_cste_FlPE_2(5.0),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10.0)

{
    setType();
    setForce();
    //ctor
}
biorbd::muscles::HillType::HillType(
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers & w,
        const biorbd::muscles::StateDynamics & s) :
    biorbd::muscles::Muscle("",g,c,w,s),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10),
    m_cste_FlPE_2(5),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10)
{
    setType();
    setForce();
    //ctor
}
biorbd::muscles::HillType::HillType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& g,
        const biorbd::muscles::Caracteristics& c,
        const biorbd::muscles::PathChangers & w,
        const biorbd::muscles::StateDynamics & s) :
    biorbd::muscles::Muscle(name,g,c,w,s),
    m_cste_FlCE_1(0.15),
    m_cste_FlCE_2(0.45),
    m_cste_FvCE_1(1),
    m_cste_FvCE_2(-.33/2 * m_cste_FvCE_1/(1+m_cste_FvCE_1)),
    m_cste_FlPE_1(10),
    m_cste_FlPE_2(5),
    m_cste_forceExcentriqueMultiplier(1.8),
    m_cste_damping(0.1),
    m_cste_vitesseRaccourMax(10)
{
    setType();
    setForce();
    //ctor
}

biorbd::muscles::HillType::HillType(const biorbd::muscles::Muscle &m):
    biorbd::muscles::Muscle(m)
{
    const biorbd::muscles::HillType& mRef(dynamic_cast<const biorbd::muscles::HillType&>(m));

    // Attributs intermédiaires lors du calcul de la force
    this->m_damping = mRef.m_damping;
    this->m_FlCE = mRef.m_FlCE;
    this->m_FlPE = mRef.m_FlPE;
    this->m_FvCE = mRef.m_FvCE;

    this->m_cste_FlCE_1 = mRef.m_cste_FlCE_1;
    this->m_cste_FlCE_2 = mRef.m_cste_FlCE_2;
    this->m_cste_FvCE_1 = mRef.m_cste_FvCE_1;
    this->m_cste_FvCE_2 = mRef.m_cste_FvCE_2;
    this->m_cste_FlPE_1 = mRef.m_cste_FlPE_1;
    this->m_cste_FlPE_2 = mRef.m_cste_FlPE_2;
    this->m_cste_forceExcentriqueMultiplier = mRef.m_cste_forceExcentriqueMultiplier;
    this->m_cste_damping = mRef.m_cste_damping;
    this->m_cste_vitesseRaccourMax = mRef.m_cste_vitesseRaccourMax;
}

biorbd::muscles::HillType::HillType(const std::shared_ptr<biorbd::muscles::Muscle> m):
    biorbd::muscles::Muscle(*m)
{
    const std::shared_ptr<biorbd::muscles::HillType> mRef(std::dynamic_pointer_cast<biorbd::muscles::HillType>(m));

    // Attributs intermédiaires lors du calcul de la force
    this->m_damping = mRef->m_damping;
    this->m_FlCE = mRef->m_FlCE;
    this->m_FlPE = mRef->m_FlPE;
    this->m_FvCE = mRef->m_FvCE;

    this->m_cste_FlCE_1 = mRef->m_cste_FlCE_1;
    this->m_cste_FlCE_2 = mRef->m_cste_FlCE_2;
    this->m_cste_FvCE_1 = mRef->m_cste_FvCE_1;
    this->m_cste_FvCE_2 = mRef->m_cste_FvCE_2;
    this->m_cste_FlPE_1 = mRef->m_cste_FlPE_1;
    this->m_cste_FlPE_2 = mRef->m_cste_FlPE_2;
    this->m_cste_forceExcentriqueMultiplier = mRef->m_cste_forceExcentriqueMultiplier;
    this->m_cste_damping = mRef->m_cste_damping;
    this->m_cste_vitesseRaccourMax = mRef->m_cste_vitesseRaccourMax;
}



biorbd::muscles::HillType::~HillType()
{
    //dtor
}

void biorbd::muscles::HillType::setType()
{
    m_type = "Hill";
}

void biorbd::muscles::HillType::setForce()
{
    m_force.clear();
    std::shared_ptr<biorbd::muscles::ForceFromOrigin> tp_o(new biorbd::muscles::ForceFromOrigin);
    std::shared_ptr<biorbd::muscles::ForceFromInsertion> tp_i(new biorbd::muscles::ForceFromInsertion);
    m_force.push_back(tp_o);
    m_force.push_back(tp_i);
    //dtor
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>>& biorbd::muscles::HillType::force(
        biorbd::rigidbody::Joints& m,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedCoordinates& Qdot,
        const biorbd::muscles::StateDynamics& EMG,
        const int updateKinLevel){
    // Update de la configuration
    if (updateKinLevel == 1)
        updateOrientations(m,Q,Qdot,false);
    else if (updateKinLevel == 2)
        updateOrientations(m,Q,Qdot,true);
    else
        biorbd::utils::Error::error(updateKinLevel == 0, "Wrong level of update in force function");

    // Calculs
    return force(EMG);
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>> &biorbd::muscles::HillType::force(
        biorbd::rigidbody::Joints &,
        const biorbd::rigidbody::GeneralizedCoordinates &,
        const biorbd::muscles::StateDynamics &,
        const int)
{
    biorbd::utils::Error::error(0, "Hill type needs velocity");
    return m_force; // Will never reach here
}

const std::vector<std::shared_ptr<biorbd::muscles::Force>> &biorbd::muscles::HillType::force(const biorbd::muscles::StateDynamics &EMG){
    // Calculer chacune les forces dans chaque éléments
    computeFvCE();
    computeFlCE(EMG);
    computeFlPE();
    computeDamping();

    // Combiner les forces
    computeForce(EMG);
    return m_force;
}

double biorbd::muscles::HillType::FlCE(const biorbd::muscles::StateDynamics &EMG)
{
    computeFlCE(EMG);
    return m_FlCE;
}

double biorbd::muscles::HillType::FlPE()
{
    computeFlPE();
    return m_FlPE;
}

double biorbd::muscles::HillType::FvCE()
{
    computeFvCE();
    return m_FvCE;
}

double biorbd::muscles::HillType::damping()
{
    computeDamping();
    return m_damping;
}

biorbd::muscles::StateDynamics biorbd::muscles::HillType::normalizeEMG(const biorbd::muscles::StateDynamics &emg){
    biorbd::muscles::StateDynamics emg_out(emg);
    emg_out.excitationNorm(caract().stateMax());
    return emg_out;
}

double biorbd::muscles::HillType::multiplyCaractByActivationAndForce(const biorbd::muscles::StateDynamics &emg){
    // Fonction qui permet de modifier la facon dont la multiplication est faite dans computeForce(EMG)
    return caract().forceIsoMax() * (emg.activation()*m_FlCE*m_FvCE + m_FlPE + m_damping);
}

void biorbd::muscles::HillType::computeForce(const biorbd::muscles::StateDynamics &EMG){
    double force = multiplyCaractByActivationAndForce(EMG);
    m_force[0]->setForce(m_position, force); // origine vers le deuxieme point
    m_force[1]->setForce(m_position, force); // insertion vers l'avant-dernier point
}

void biorbd::muscles::HillType::computeFlPE(){
	if (m_position.length() > caract().tendonSlackLength())
		m_FlPE = exp(m_cste_FlPE_1*(m_position.length()/caract().optimalLength()-1) - m_cste_FlPE_2);
	else 
		m_FlPE = 0;
}

void biorbd::muscles::HillType::computeDamping(){
    m_damping = m_position.velocity() / (m_cste_vitesseRaccourMax * caract().optimalLength()) * m_cste_damping;
}

void biorbd::muscles::HillType::computeFlCE(const biorbd::muscles::StateDynamics &EMG){
    m_FlCE = exp( -pow(( m_position.length() / caract().optimalLength() / (m_cste_FlCE_1*(1-EMG.activation())+1) -1 ), 2)/m_cste_FlCE_2   );
}

void biorbd::muscles::HillType::computeFvCE(){
    // La relation est différente si la vitesse < 0  ou > 0
    double v = m_position.velocity();
    if (v<=0)
        m_FvCE = (1-abs(v)/m_cste_vitesseRaccourMax) /
            (1+abs(v)/m_cste_vitesseRaccourMax/m_cste_FvCE_1);
    else
        m_FvCE = (1-1.33*v/m_cste_vitesseRaccourMax/m_cste_FvCE_2) /
                   (1-v/m_cste_vitesseRaccourMax/m_cste_FvCE_2);
}






